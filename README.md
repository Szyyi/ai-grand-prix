# AI Grand Prix — Autonomous Drone Racing Stack

## Architecture

```
┌─────────────────────────────────────────────────┐
│                  MAIN LOOP                       │
│  sense() → plan() → control() → actuate()       │
└──────┬──────────┬──────────┬──────────┬──────────┘
       │          │          │          │
  ┌────▼───┐ ┌───▼────┐ ┌──▼───┐ ┌───▼─────┐
  │ State  │ │  Path  │ │ PID/ │ │ Comms   │
  │ Estim. │ │Planner │ │ MPC  │ │Interface│
  └────────┘ └────────┘ └──────┘ └─────────┘
```

## Modules

- **core/** — Drone state, gate representation, race manager
- **planning/** — Trajectory generation, gate sequencing, spline paths
- **control/** — PID controller, optional MPC, velocity/attitude control
- **simulation/** — Local physics sim, racecourse generator, visualiser
- **stress_tests/** — Automated edge-case and reliability testing
- **config/** — Tuning parameters, course definitions
- **utils/** — Math helpers, logging, telemetry recording

## Design Principles

1. **Reliability over speed** — Complete all gates first, optimise time second
2. **Modular I/O** — Competition interface is a thin adapter; core logic is decoupled
3. **Deterministic replay** — Every run is logged and reproducible
4. **Stress-tested** — Automated tests for wind, gate misalignment, physics edge cases

## Quick Start

```bash
pip install numpy scipy matplotlib
python -m simulation.run_race --course default --visualise
python -m stress_tests.run_all
```