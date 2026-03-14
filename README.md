# Resilient Control Strategies for Autonomous Vehicle Propulsion

Residual-based fault detection and adaptive control reconfiguration for 
autonomous vehicle propulsion systems — implemented in Python with 
MATLAB visualization.

---

## Key Results

- **30%** automatic speed reduction upon fault detection for safe operation
- Real-time fault detection via configurable residual threshold
- Eigenvalue-based stability verification with adaptive gain adjustment
- Validated across multiple fault injection scenarios

---

## System Overview

| Component | Implementation |
|-----------|---------------|
| Vehicle Dynamics | Kinematic motion model in Python |
| Fault Detection | Residual-based threshold monitoring |
| Control Reconfiguration | Automatic speed/angular velocity adjustment |
| Stability Analysis | Observability, controllability, eigenvalue verification |
| Fault Accommodation | Adaptive gain adjustment to maintain stability |
| Visualization | Python-MATLAB engine for trajectory and residual plots |

---

## Repo Layout
```
resilient-control/
├── src/
│   ├── main.py                  # Vehicle dynamics + fault detection
│   ├── fault_accommodation.py   # Observability + gain adjustment
│   └── plotting.py              # MATLAB engine visualization
├── matlab/
│   └── plot_results.m           # MATLAB plotting scripts
├── results/                     # Simulation output plots
├── docs/                        # Project report and diagrams
├── requirements.txt
└── README.md
```

---

## Quick Start
```bash
# Clone repo
git clone https://github.com/Shubh131102/Resilient-Control-Autonomous-Vehicle-Propulsion.git
cd Resilient-Control-Autonomous-Vehicle-Propulsion

# Install dependencies
pip install -r requirements.txt

# Run vehicle dynamics simulation with fault detection
python src/main.py

# Run fault accommodation and stability analysis
python src/fault_accommodation.py
```

---

## How It Works

### Fault Detection

Vehicle state is propagated via kinematic model at each timestep.
Sensor measurements include Gaussian noise — larger noise injected at fault time.
Residual computed as norm of difference between measured and predicted state.
If residual exceeds threshold, fault is flagged and control is reconfigured.
```
Residual = ||measured_state - predicted_state||
If residual > threshold → Fault detected → Reconfigure control
```

### Control Reconfiguration

Upon fault detection:
- Linear velocity reduced by **30%**
- Angular velocity adjusted for stability
- System continues operating safely under degraded conditions

### Stability Analysis

State-space system analyzed for:
- **Observability** — full rank observability matrix confirms state estimation possible
- **Controllability** — full rank controllability matrix confirms system stabilizable
- **Eigenvalue verification** — closed-loop eigenvalues checked for negative real parts
- **Adaptive gain** — control gain K adjusted if instability detected

---

## Simulation Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| dt | 0.1s | Time step |
| sim_time | 20s | Total simulation duration |
| fault_time | 10s | Fault injection time |
| residual_threshold | 0.7 | Fault detection threshold |
| normal_noise | σ=0.1 | Sensor noise standard deviation |
| fault_noise | σ=2.0 | Faulty sensor noise standard deviation |
| speed_reduction | 30% | Control reconfiguration on fault |

---

## Results

Trajectory and residual plots generated after simulation:

- **Trajectory plot** — vehicle path before and after fault injection
- **Residual plot** — residual over time with threshold line
- Fault detection timestamp printed to console with residual value

---

## Requirements
```
numpy>=1.19
matplotlib>=3.3
scipy>=1.7
```

Optional for MATLAB visualization:
- MATLAB R2019b or later
- MATLAB Engine API for Python

Install Python dependencies:
```bash
pip install -r requirements.txt
```

---

## Notes

- Fault injection time configurable via sensor_fault_time parameter
- Residual threshold tunable for different noise environments
- MATLAB visualization optional — matplotlib fallback available
- State-space matrices A, B, C configurable for different system models
