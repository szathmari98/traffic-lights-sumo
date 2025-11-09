# SUMO Traffic Light Controllers – Adaptive • Predictive • Cooperative

This repository contains three simple, production-ready traffic light controllers for **SUMO** using **TraCI**:

- `adaptive.py` – adjusts green time by current queue length (threshold rule).
- `predictive.py` – adjusts by **queue trend** over a sliding window.
- `cooperative.py` – **phase-level soft cooperation** using ETA-weighted local+upstream load.

All three scripts are standalone, readable, and tuned for clarity + reproducibility.

---

## Requirements

- **SUMO** (Simulation of Urban MObility) installed  
  - Set `SUMO_HOME` and ensure `sumo` is on PATH.
- **Python 3.9+**
- `traci` (provided by SUMO; alternatively `pip install traci`)

Controllers – parameters & logic
1) adaptive_lights.py

Idea: if avg_queue > UPPER_THRESHOLD → increase green (INCREMENT),
if < LOWER_THRESHOLD → decrease green (DECREMENT).

Periodic reset to base durations (RESET_INTERVAL) to avoid drift.

Bounds: MIN_GREEN … MAX_GREEN.

Key params:
