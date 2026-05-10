
# LunarAscent – 6-DOF Launch Vehicle GNC Simulation

**A complete guidance, navigation, and control simulation for multi-stage launch vehicles from liftoff to orbit insertion and trans-lunar injection.**

![Version](https://img.shields.io/badge/version-1.0.0-blue)
![Language](https://img.shields.io/badge/language-C%2B%2B17-orange)
![License](https://img.shields.io/badge/license-MIT-green)
![Simulation](https://img.shields.io/badge/simulation-6--DOF-red)
![Status](https://img.shields.io/badge/status-Operational-brightgreen)

---

## Overview

**LunarAscent** is a high-fidelity 6-degree-of-freedom launch vehicle simulator that models the complete ascent phase from liftoff through orbit insertion. It implements a full GNC pipeline: gravity turn guidance, inertial navigation with GPS aiding, and 3-axis attitude control via engine gimbal and RCS thrusters.

The vehicle model is based on the Falcon 9 v1.2 with two stages, 9 Merlin engines, and realistic thrust curves from sea level to vacuum. The simulation captures Max-Q throttling, stage separation, fairing jettison, and orbital injection with configurable target orbit parameters.

---

## Key Capabilities

| Subsystem | Implementation |
|-----------|----------------|
| **Vehicle Dynamics** | 6-DOF rigid body with quaternion attitude, variable mass, and inertia tensor |
| **Propulsion** | Multi-engine model with altitude-dependent thrust/Isp, gimbal limits, throttle ramping |
| **Environment** | WGS84 gravity with J2, ISA atmosphere to 86 km, Mach-dependent aerodynamics |
| **Guidance** | Gravity turn + pitch program + closed-loop orbital guidance + TLI targeting |
| **Navigation** | INS with IMU noise models, GPS aiding via Kalman filter, star tracker |
| **Control** | Cascade PID with gain scheduling (Mach-based), anti-windup, RCS for roll |
| **Visualization** | Python/Matplotlib trajectory plotter with event detection |

---

## Project Structure

```
LunarAscent/
├── include/
│   ├── vehicle.hpp              # Vehicle state, quaternion, aerodynamics
│   ├── propulsion.hpp           # Engine model, RCS, thrust vector control
│   ├── environment.hpp          # Gravity, atmosphere, wind, aerodynamic forces
│   ├── guidance.hpp             # Pitch program, gravity turn, orbit targeting
│   ├── navigation.hpp           # INS, GPS, Kalman filter, coordinate transforms
│   └── control.hpp              # PID controller, gain scheduling, throttle logic
├── src/
│   ├── vehicle.cpp
│   ├── propulsion.cpp
│   ├── environment.cpp
│   ├── guidance.cpp
│   ├── navigation.cpp
│   ├── control.cpp
│   └── main.cpp                 # Full simulation with Falcon 9 configuration
├── sim/
│   └── plot_trajectory.py       # 6-panel trajectory visualization
├── Makefile
├── README.md
└── LICENSE
```

---

## Quick Start

### Prerequisites

- **GCC 9+** or **Clang 10+** with C++17 support
- **Python 3.8+** with NumPy and Matplotlib (for visualization)

### Build

```bash
make
```

### Run Simulation

```bash
make run
```

This generates `trajectory.csv` with full flight data at 100 Hz resolution.

### Plot Results

```bash
make plot
```

Generates `trajectory_profile.png` with altitude, velocity, acceleration, Mach, pitch, and mass plots.

---

## Vehicle Configuration

| Parameter | Stage 1 | Stage 2 |
|-----------|---------|---------|
| Dry Mass | 25,600 kg | 3,900 kg |
| Propellant | 395,700 kg | 92,670 kg |
| Thrust (vac) | 8,227 kN | 934 kN |
| Isp (vacuum) | 348 s | 348 s |
| Burn Time | 162 s | 397 s |
| Engines | 9 × Merlin 1D | 1 × Merlin 1D Vac |
| Gimbal Range | ±8° | ±8° |

---

## Flight Phases

| Phase | Description |
|-------|-------------|
| **VERTICAL_ASCENT** | First ~500m, vertical climb |
| **PITCH_OVER** | Gradual tilt to initiate gravity turn |
| **GRAVITY_TURN** | Natural trajectory following velocity vector |
| **MAX_Q** | Throttle down to limit dynamic pressure |
| **OPEN_LOOP_PITCH** | Pre-programmed pitch profile |
| **CLOSED_LOOP_GUIDANCE** | Active targeting of insertion orbit |
| **ORBIT_INJECTION** | Final circularization burn |
| **COAST** | Ballistic coast phase |

---

## Sample Output

```
[Plot] Loaded 6000 data points
Time range: 0.0s - 580.0s
Max altitude: 208.5 km
Max velocity: 7792.3 m/s
Max Mach: 6.42
Max Q: 32.8 kPa
Max acceleration: 4.2g
Final mass: 5230 kg

Detected 4 mission events:
  T+68.5s: Max-Q: 32.8 kPa
  T+162.0s: Stage 1 Cutoff (MECO)
  T+165.0s: Stage 2 Ignition
  T+520.0s: Orbit Insertion
```

---

## Physics Models

- **Gravity**: WGS84 ellipsoid with J2 oblateness perturbation
- **Atmosphere**: ISA standard atmosphere with 8-layer temperature/pressure model
- **Aerodynamics**: Mach-dependent drag coefficient with table lookup
- **Thrust**: Linear interpolation between sea-level and vacuum thrust/Isp
- **Gimbal**: Rate-limited actuator (±5°/s) with ±8° hard stops
- **IMU**: Gyro angular random walk + bias drift; accelerometer velocity random walk

---

## License

MIT License – See `LICENSE` file for details.

---
