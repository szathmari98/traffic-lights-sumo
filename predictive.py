#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Predictive traffic light control for SUMO.
Adjusts green phase duration based on the trend of queue length (increasing/decreasing).
"""

import traci
import traci.constants as tc
import collections
import os

# --- configurable parameters ---
SUMO_BINARY = os.environ.get("SUMO_BINARY", "sumo")
CONFIG_FILE = "bkk_predictive.sumocfg"

WINDOW_SIZE = 10          # number of steps used for trend calculation
INC = 5                   # seconds to increase green time
DEC = 4                   # seconds to decrease green time
MIN_GREEN = 10            # minimum green duration
MAX_GREEN = 60            # maximum green duration
STEP_LENGTH = 1.0         # simulation step size (seconds)

print("Predictive traffic light control starting...")
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
queue_history = {tls: collections.deque(maxlen=WINDOW_SIZE) for tls in tls_ids}
step = 0

while step < 3600:
    traci.simulationStep()
    step += 1

    for tls in tls_ids:
        # gather queue data for all lanes controlled by the signal
        controlled_lanes = traci.trafficlight.getControlledLanes(tls)
        queues = [traci.lane.getLastStepHaltingNumber(l) for l in controlled_lanes]
        avg_queue = sum(queues) / len(queues) if queues else 0
        queue_history[tls].append(avg_queue)

        # wait until enough history is collected
        if len(queue_history[tls]) < WINDOW_SIZE:
            continue

        # calculate queue trend between first and second half of window
        first_half = list(queue_history[tls])[:WINDOW_SIZE // 2]
        second_half = list(queue_history[tls])[WINDOW_SIZE // 2:]
        trend = (sum(second_half) / len(second_half)) - (sum(first_half) / len(first_half))

        base_green = max([d for (t, _), d in tls_base_green.items() if t == tls])
        gtime = green_times[tls]

        # adjust green time according to the observed trend
        if trend > 0.5:               # queue increasing
            gtime = min(gtime + INC, MAX_GREEN)
        elif trend < -0.5:            # queue decreasing
            gtime = max(gtime - DEC, MIN_GREEN)
        else:                         # small fluctuations → return toward base
            if gtime > base_green:
                gtime -= 1
            elif gtime < base_green:
                gtime += 1

        green_times[tls] = gtime

        # apply updated durations to all green phases
        logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(tls)[0]
        for ph in logic.getPhases():
            if "G" in ph.state:
                ph.duration = gtime
        traci.trafficlight.setCompleteRedYellowGreenDefinition(tls, logic)

    # print progress every 600 simulation steps
    if step % 600 == 0:
        print(f"Step {step}: {[ (tls, round(green_times[tls],1)) for tls in tls_ids ]}")

traci.close()
print("Predictive finished.")
