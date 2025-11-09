#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase-level cooperative traffic light control for SUMO (TraCI interface).
This controller adjusts green durations by considering both local and upstream traffic loads,
using ETA-based weighting and smooth ("soft") cooperation between intersections.
"""

import warnings
warnings.filterwarnings("ignore", category=UserWarning)

import os
import time
from math import inf
from collections import defaultdict
import traci

# --- environment / configuration ---
SUMO_BINARY  = os.environ.get("SUMO_BINARY", "sumo")
CONFIG_FILE  = "bkk_cooperative.sumocfg"

# --- cooperation horizon parameters ---
USE_TIME_HORIZON = True
H_TIME           = 9.0     # s – shorter horizon to avoid overshoot
H_DISTANCE       = 250.0   # m – used if distance-based ETA is applied

ALPHA_UPSTREAM   = 0.11    # influence weight of the immediate upstream intersection
ETA_DECAY        = 0.6     # ETA-based weight decay: weight = 1 / (1 + (eta/H_TIME)^ETA_DECAY)

# --- green time adjustment rules ---
INC              = 3        # s – increment step
DEC              = 1        # s – decrement step
RELAX_RATE       = 2        # s – relaxation speed toward base green in medium load range
MIN_GREEN        = 8
MAX_GREEN        = 60
LOW_LOAD_CUTOFF  = 1.0      # below this, decrease toward base
HIGH_LOAD_CUTOFF = 5.0      # above this, increase green time

# --- simulation timing ---
STEP_LENGTH      = 1.0
STEP_LIMIT       = 3600
UPDATE_PERIOD    = 2.0      # s – minimum interval between adjustments

# -------------------- helper functions --------------------

def lane_to_edge(lane_id: str) -> str:
    """Convert a SUMO lane ID (e.g. 'edge_0') to its parent edge name ('edge')."""
    if not lane_id:
        return None
    return lane_id.rsplit('_', 1)[0]

def build_phase_approaches(tls_id):
    """
    Determine which incoming edges are served by each phase.
    Uses controlled link indices (from getControlledLinks) combined with the phase state string.
    """
    logic  = traci.trafficlight.getCompleteRedYellowGreenDefinition(tls_id)[0]
    phases = logic.getPhases()
    links  = traci.trafficlight.getControlledLinks(tls_id)

    in_lanes = []
    for alts in links:
        if alts and alts[0] and len(alts[0]) >= 1:
            in_lanes.append(alts[0][0])
        else:
            in_lanes.append(None)

    phase_to_edges = {}
    for p_idx, ph in enumerate(phases):
        served_edges = set()
        state = ph.state
        for k, ch in enumerate(state):
            if ch in ("G", "g") and k < len(in_lanes) and in_lanes[k]:
                e = lane_to_edge(in_lanes[k])
                if e:
                    served_edges.add(e)
        phase_to_edges[p_idx] = served_edges

    return phase_to_edges, phases

def extract_base_greens(phases):
    """Extract base (default) green durations for all green phases."""
    base = {}
    for i, ph in enumerate(phases):
        if any(c in ("G", "g") for c in ph.state):
            base[i] = float(ph.duration)
    return base

def eta_ok(dist, speed):
    """Determine if a vehicle is within the cooperation horizon based on ETA or distance."""
    if USE_TIME_HORIZON:
        v = max(speed, 0.1)
        eta = dist / v if dist != inf else inf
        return eta <= H_TIME, eta
    else:
        return (dist <= H_DISTANCE), None

def weight_by_eta(eta):
    """Compute ETA-based weight for vehicles approaching within the cooperation horizon."""
    if eta is None:
        return 1.0
    x = max(eta / max(H_TIME, 0.1), 0.0)
    return 1.0 / (1.0 + (x ** ETA_DECAY))

# -------------------- main control loop --------------------

def main():
    print("Cooperative traffic control starting…")
    traci.start([SUMO_BINARY, "-c", CONFIG_FILE, "--step-length", str(STEP_LENGTH)])

    tls_ids = list(traci.trafficlight.getIDList())
    print(f"Found {len(tls_ids)} TLS: {tls_ids}")

    # Precompute per-TLS phase mappings and base green durations
    phase_edges   = {}
    base_green    = {}
    target_green  = {}      # (tls, phase) -> current target green
    last_update   = {}      # (tls, phase) -> last adjustment time
    last_phase_id = {}      # tls -> for detecting phase changes
    phase_start_t = {}      # tls -> phase start time

    for tls in tls_ids:
        pe, phases = build_phase_approaches(tls)
        phase_edges[tls] = pe
        bg = extract_base_greens(phases)
        if not bg:  # fallback if no green phase found
            for i in range(len(phases)):
                if any(c in ("G", "g") for c in phases[i].state):
                    bg[i] = 30.0
        base_green[tls] = bg
        for p, g in bg.items():
            target_green[(tls, p)] = max(MIN_GREEN, min(MAX_GREEN, g))
            last_update[(tls, p)]  = -1e9

        # initialize phase tracking
        p = traci.trafficlight.getPhase(tls)
        last_phase_id[tls] = p
        phase_start_t[tls] = traci.simulation.getTime()

        # initialization log
        for p, g in sorted(bg.items()):
            print(f"[INIT] {tls} phase {p}: base green = {g:.1f}s, edges={sorted(list(phase_edges[tls][p]))}")

    step = 0
    while step < STEP_LIMIT:
        traci.simulationStep()
        step += 1
        sim_t = traci.simulation.getTime()

        # --- load measurement ---
        # 1) Local: vehicles approaching current TLS on edges served by its current green phase
        # 2) Upstream: vehicles for which the next TLS downstream is this one (weighted by ETA)
        local_load_per_tls_phase = defaultdict(lambda: defaultdict(float))
        upstream_load_per_tls    = defaultdict(float)

        vids = traci.vehicle.getIDList()
        for vid in vids:
            nxt = traci.vehicle.getNextTLS(vid)
            if not nxt:
                continue

            spd = traci.vehicle.getSpeed(vid)

            # Local contribution
            tls0, dist0, _, _ = nxt[0]
            if tls0 in tls_ids:
                ok, eta0 = eta_ok(dist0, spd)
                if ok:
                    lane  = traci.vehicle.getLaneID(vid)
                    edge0 = lane_to_edge(lane)
                    if edge0:
                        for p, edges in phase_edges[tls0].items():
                            if edge0 in edges:
                                w = weight_by_eta(eta0)
                                local_load_per_tls_phase[tls0][p] += w

            # Upstream contribution (only for directly connected TLS pairs)
            if len(nxt) >= 2:
                tls_up, dist_up, _, _ = nxt[0]
                tls_dw, _, _, _ = nxt[1]
                if tls_up in tls_ids and tls_dw in tls_ids:
                    ok_up, eta_up = eta_ok(dist_up, spd)
                    if ok_up:
                        upstream_load_per_tls[tls_dw] += weight_by_eta(eta_up)

        # --- control decision: only modify the currently active green phase ---
        for tls in tls_ids:
            p_cur = traci.trafficlight.getPhase(tls)
            state_cur = traci.trafficlight.getRedYellowGreenState(tls)

            # detect phase change
            if p_cur != last_phase_id[tls]:
                last_phase_id[tls] = p_cur
                phase_start_t[tls] = sim_t

            # skip non-green or special phases
            if not any(c in ("G", "g") for c in state_cur):
                continue
            if p_cur not in base_green[tls]:
                continue

            # rate limit adjustments
            if sim_t - last_update[(tls, p_cur)] < UPDATE_PERIOD:
                continue

            base_g   = base_green[tls][p_cur]
            target_g = target_green[(tls, p_cur)]
            elapsed  = sim_t - phase_start_t[tls]

            # calculate combined load
            local_load = local_load_per_tls_phase[tls].get(p_cur, 0.0)
            up_load    = upstream_load_per_tls.get(tls, 0.0)
            weighted   = local_load + ALPHA_UPSTREAM * up_load

            # --- adjustment logic ---
            if weighted >= HIGH_LOAD_CUTOFF:
                target_g = min(target_g + INC, MAX_GREEN)
            elif weighted <= LOW_LOAD_CUTOFF:
                if target_g > base_g:
                    target_g = max(target_g - DEC, base_g)
                else:
                    target_g = max(target_g - 1, MIN_GREEN)
            else:
                if target_g > base_g:
                    target_g = max(target_g - RELAX_RATE, base_g)
                elif target_g < base_g:
                    target_g = min(target_g + RELAX_RATE, base_g)

            remaining = max(target_g - elapsed, 0.5)
            traci.trafficlight.setPhaseDuration(tls, remaining)

            target_green[(tls, p_cur)] = target_g
            last_update[(tls, p_cur)]  = sim_t

        # periodic debug output
        if step % 300 == 0:
            snap = []
            for tls in tls_ids[:6]:
                p = traci.trafficlight.getPhase(tls)
                tg = target_green.get((tls, p), None)
                if tg is not None:
                    snap.append(f"{tls}:P{p}->{tg:.0f}s")
            print(f"t={int(sim_t)}s  targets: {', '.join(snap)}")

    traci.close()
    print("Cooperative finished.")


if __name__ == "__main__":
    main()
