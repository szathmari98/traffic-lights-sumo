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
