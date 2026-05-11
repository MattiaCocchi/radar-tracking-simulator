# Radar Tracking Simulator — C++17

A lightweight, dependency-free C++ simulator that models the full pipeline of a radar tracking system: a moving entity, a noisy radar sensor, and two competing filters — a classic α-filter and a **Kalman filter with velocity estimation** — shown side by side on every time-step.

```
=== Aircraft A  (vx=10 m/s, vy=2 m/s) ===
   t  Real X Radar X  α-filt  Kalman   KalVx |err_α|  |err_K|
--------------------------------------------------------------
  5s   50.00   47.08   25.70   50.04   10.35   24.30     0.04
 10s  100.00   96.52   63.34   97.10    9.73   36.66     2.90
 20s  200.00  200.06  159.25  198.68   10.02   40.75     1.32
 30s  300.00  300.13  259.73  300.03   10.12   40.27     0.03
--------------------------------------------------------------
Mean |err| X →  α-filter:   34.60m    Kalman:    1.57m
```

---

## Architecture

```
radar_sim/
├── core/
│   ├── Entity.h / .cpp          # Kinematic ground truth (constant-velocity model)
│   ├── Radar.h  / .cpp          # Sensor: adds Gaussian noise (std::normal_distribution)
│   ├── KalmanFilter1D.h / .cpp  # 2-state CV Kalman filter (pos + vel), one axis
│   ├── Tracker.h / .cpp         # 2-D tracker: two KalmanFilter1D + legacy α-filter
│   └── Logger.h                 # CSV export for post-processing / plotting
├── main.cpp                     # SimulationRunner<T> template, two demo scenarios
└── CMakeLists.txt
```

### Module responsibilities

| Module | Role |
|---|---|
| **Entity** | Maintains the real kinematic state `{x, y, vx, vy}` via Euler integration. This is the *ground truth* — never seen by the tracker. |
| **Radar** | Samples the true position and corrupts it with zero-mean Gaussian noise `N(0, σ²)`, simulating real sensor error. |
| **KalmanFilter1D** | Single-axis, 2-state filter. State `[pos, vel]`. Predict step propagates with `F = [[1,dt],[0,1]]`; update step applies the optimal Kalman gain. No external libraries. |
| **Tracker** | Owns two `KalmanFilter1D` instances (X and Y axes, decoupled). Also runs a legacy α-filter in parallel for direct error comparison. |
| **Logger** | Writes every time-step to CSV so results can be plotted in Python, Qt Charts, or Excel. |

---

## Why the Kalman filter beats the α-filter

An α-filter is a **zeroth-order estimator**: it has no internal model of velocity.  
When tracking a target moving at constant velocity `v`, it accumulates a steady-state lag:

```
lag_α ≈ ((1 − α) / α) · v · T
```

With `α = 0.2`, `v = 10 m/s`, `T = 1 s` → **lag ≈ 40 m** — exactly what the output shows.

The **Constant-Velocity Kalman** carries velocity in its state vector. After a short convergence transient it predicts the next position from the estimated velocity, eliminating the lag almost entirely.

| Scenario | α-filter mean error | Kalman mean error |
|---|---|---|
| Slow aircraft  (vx = 10 m/s) | 34.6 m | **1.6 m** |
| Fast aircraft  (vx = 50 m/s) | 167.1 m | **1.5 m** |

The Kalman error is bounded by the radar noise floor regardless of target speed; the α-filter error grows linearly with velocity.

---

## Kalman filter internals

The filter operates independently on each axis with state `x = [pos, vel]ᵀ`.

### Predict step
```
F = [[1, dt],      x_pred = F · x
     [0,  1]]      P_pred = F · P · Fᵀ + Q
```

### Update step
```
H = [1, 0]                         (we only measure position)
S = H · P_pred · Hᵀ + R            (innovation covariance, scalar)
K = P_pred · Hᵀ / S                (Kalman gain)
x = x_pred + K · (z − H · x_pred) (state correction)
P = (I − K · H) · P_pred           (covariance correction)
```

**Tuning parameters** (constructor of `Tracker`):

| Parameter | Meaning | Increase when… |
|---|---|---|
| `sigma_meas` | Radar noise std-dev [m] | sensor is less accurate |
| `q_pos` | Process noise — position | target manoeuvres unpredictably |
| `q_vel` | Process noise — velocity | target can change speed quickly |

---

## Bugs fixed from original code

Two silent bugs in the original `Entity.cpp` caused all position outputs to be **always zero**:

```cpp
//  BEFORE — absolute assignment: entity never moved
e_state.x = e_state.vx * dt;

//  AFTER — Euler integration
e_state.x += e_state.vx * dt;
```

```cpp
// BEFORE — returned a default-constructed (all-zero) State
State Entity::getState() const { return State(); }

// AFTER
State Entity::getState() const { return e_state; }
```

---

## Build

**Requirements:** C++17 compiler, CMake ≥ 3.16. No external dependencies.

---

## Output

The program prints a comparison table to stdout and writes two CSV files:

| File | Content |
|---|---|
| `aircraft_slow.csv` | 30-step log for vx=10 m/s scenario |
| `aircraft_fast.csv` | 30-step log for vx=50 m/s scenario |

Each CSV contains: `t, real_x, real_y, radar_x, radar_y, kalman_x, kalman_y, kalman_vx, kalman_vy, alpha_x, alpha_y`

### Quick plot with Python
```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("aircraft_slow.csv")
plt.plot(df.t, df.real_x,    label="Ground truth")
plt.plot(df.t, df.radar_x,   label="Radar (noisy)", alpha=0.5)
plt.plot(df.t, df.kalman_x,  label="Kalman estimate")
plt.plot(df.t, df.alpha_x,   label="α-filter estimate")
plt.legend(); plt.xlabel("Time [s]"); plt.ylabel("X position [m]")
plt.show()
```

---

## Roadmap

- [ ] **Extended Kalman Filter (EKF)** — support for non-linear motion models (turns, acceleration)  
- [ ] **Multi-target tracking** — `SimulationRunner<T>` template already supports N entities; add data association (nearest-neighbour or JPDA)  
- [ ] **Qt6 GUI** — connect `Tracker::GetEstimatedState()` to a `Signal/Slot` chain; render three moving dots (truth / radar / estimate) in a `QGraphicsScene` at real time  
- [ ] **Unit tests** — GoogleTest suite for predict/update steps with deterministic seeds  
- [ ] **Benchmark mode** — Monte Carlo over 1000 runs to compute RMSE curves vs σ and α  

---

## License

MIT