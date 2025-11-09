#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Bidirectional adaptive traffic light control for SUMO.
Adjusts green phase duration dynamically based on queue length.
"""

import traci
import traci.constants as tc
import os

# --- configurable parameters ---
SUMO_BINARY = os.environ.get("SUMO_BINARY", "sumo")
CONFIG_FILE = "bkk_adaptive.sumocfg"

UPPER_THRESHOLD = 8       # queue length above which green time increases
LOWER_THRESHOLD = 2       # queue length below which green time decreases
INCREMENT = 5             # seconds to increase green time
DECREMENT = 4             # seconds to decrease green time
MIN_GREEN = 10            # minimum green duration
MAX_GREEN = 60            # maximum green duration
RESET_INTERVAL = 300      # interval (steps) after which green time resets to base
STEP_LENGTH = 1.0         # simulation step size (seconds)

print("Adaptive traffic light control starting...")
traci.start([SUMO_BINARY, "-c", CONFIG_FILE])

tls_ids = traci.trafficlight.getIDList()
print(f"Found {len(tls_ids)} traffic lights: {tls_ids}")

# --- read and store base green durations dynamically ---
tls_base_green = {}
for tls in tls_ids:
    logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(tls)[0]
    phases = logic.getPhases()
    for i, ph in enumerate(phases):
        if "G" in ph.state:
            tls_base_green[(tls, i)] = ph.duration
            print(f"[INIT] {tls} phase {i}: base green = {ph.duration:.1f}s")

# fallback if no green phases are found
if not tls_base_green:
    print("No green phases detected, using default 30s base.")
    for tls in tls_ids:
        tls_base_green[(tls, 0)] = 30

green_times = {tls: max([d for (t, _), d in tls_base_green.items() if t == tls]) for tls in tls_ids}
step = 0

while step < 3600:
    traci.simulationStep()
    step += 1

    for tls in tls_ids:
        controlled_lanes = traci.trafficlight.getControlledLanes(tls)
        queues = [traci.lane.getLastStepHaltingNumber(l) for l in controlled_lanes]
        avg_queue = sum(queues) / len(queues) if queues else 0

        base_green = max([d for (t, _), d in tls_base_green.items() if t == tls])
        gtime = green_times[tls]

        # adjust green time based on queue thresholds
        if avg_queue > UPPER_THRESHOLD:
            gtime = min(gtime + INCREMENT, MAX_GREEN)
        elif avg_queue < LOWER_THRESHOLD:
            gtime = max(gtime - DECREMENT, MIN_GREEN)

        # periodically reset to base duration
        if step % RESET_INTERVAL == 0:
            gtime = base_green

        green_times[tls] = gtime

        # apply updated phase durations
        logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(tls)[0]
        for ph in logic.getPhases():
            if "G" in ph.state:
                ph.duration = gtime
        traci.trafficlight.setCompleteRedYellowGreenDefinition(tls, logic)

    # print intermediate results periodically
    if step % 600 == 0:
        print(f"Step {step}: {[ (tls, round(green_times[tls],1)) for tls in tls_ids ]}")

traci.close()
print("Adaptive finished.")
