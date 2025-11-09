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

## Controllers – parameters & logic
Key params:

| Parameter     | Default | Description                  |
| ------------- | ------- | ---------------------------- |
| `MIN_GREEN`   | 10 s    | Minimum green phase duration |
| `MAX_GREEN`   | 60 s    | Maximum green phase duration |
| `STEP_LENGTH` | 1.0 s   | Simulation step size         |
| `STEP_LIMIT`  | 3600    | Simulation duration in steps |

Adaptive Controller (adaptive_lights.py)
| Parameter         | Default | Description                                  |
| ----------------- | ------- | -------------------------------------------- |
| `UPPER_THRESHOLD` | 8       | If average queue > threshold, increase green |
| `LOWER_THRESHOLD` | 2       | If average queue < threshold, decrease green |
| `INCREMENT`       | 5 s     | Green extension step                         |
| `DECREMENT`       | 4 s     | Green reduction step                         |
| `RESET_INTERVAL`  | 300 s   | Periodic reset to base duration              |

Predictive Controller (predictive_lights.py)
| Parameter         | Default | Description                                           |
| ----------------- | ------- | ----------------------------------------------------- |
| `WINDOW_SIZE`     | 10      | Sliding window of recent queue values                 |
| `INC` / `DEC`     | 5 / 4 s | Step size for extending/reducing green                |
| `trend` threshold | ±0.5    | Sensitivity to queue growth/decline                   |
| Behavior          | –       | Returns gradually toward base green if no clear trend |

Cooperative Controller (cooperative_lights_v3.py)
| Parameter              | Default   | Description                             |
| ---------------------- | --------- | --------------------------------------- |
| `H_TIME`               | 9.0 s     | Time horizon for ETA-based cooperation  |
| `ALPHA_UPSTREAM`       | 0.11      | Influence of upstream intersection load |
| `INC` / `DEC`          | 3 / 1 s   | Adjustment step sizes                   |
| `RELAX_RATE`           | 2 s       | Return speed toward base green          |
| `LOW/HIGH_LOAD_CUTOFF` | 1.0 / 5.0 | Activation thresholds                   |
| `UPDATE_PERIOD`        | 2.0 s     | Minimum delay between two updates       |

## References

Li, L. et al., Traffic Signal Timing via Deep Reinforcement Learning, IEEE/CAA Journal of Automatica Sinica, 2016.
Badii, C. et al., Sii-Mobility: An IoT/IoE Architecture to Enhance Smart City Mobility and Transportation Services, Sensors, 2019.
BKK Budapest, Hogyan és milyen forgalmi adatokat gyűjt a BKK, 2023.


Feel free to reuse and modify.
