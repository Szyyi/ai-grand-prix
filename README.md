<div align="center">

# AI GRAND PRIX — Autonomous Drone Racing Stack

### A Competition-Grade Autonomy Framework for the Anduril AI Grand Prix

**Status:** Active Development · **Version:** 0.2.0 · **License:** Proprietary

[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-blue.svg)]()
[![Tests](https://img.shields.io/badge/Stress_Tests-30%2F30_Passing-brightgreen.svg)]()
[![Architecture](https://img.shields.io/badge/Architecture-Modular_Pipeline-orange.svg)]()

---

*Built to win. Every module, every line, every decision optimised for one goal:*
*fastest valid completion of all gates under realistic physical constraints.*

</div>

---

## Table of Contents

- [Mission Statement](#mission-statement)
- [System Architecture](#system-architecture)
- [Module Breakdown](#module-breakdown)
- [Technical Deep Dive](#technical-deep-dive)
- [Performance Benchmarks](#performance-benchmarks)
- [Development Roadmap](#development-roadmap)
- [Quick Start](#quick-start)
- [Usage Guide](#usage-guide)
- [Design Philosophy](#design-philosophy)
- [Competition Adapter Strategy](#competition-adapter-strategy)
- [Contributing](#contributing)

---

## Mission Statement

The AI Grand Prix, hosted by Anduril Industries and the Drone Champions League (DCL), is an international autonomous drone racing competition. Teams submit Python-based autonomy stacks that are evaluated inside a physics-realistic simulator on precision, stability, and speed.

This repository contains a **complete, modular, stress-tested autonomy framework** designed from day one to:

1. **Guarantee gate completion** — reliability is the prerequisite; speed is the optimisation target.
2. **Decouple intelligence from interface** — when the competition platform drops, we swap one file, not the stack.
3. **Win under adversarial conditions** — wind, noise, small gates, degraded motors. If it works in hell, it works on race day.

---

## System Architecture

### High-Level Pipeline

```
+=====================================================================+
|                        AUTONOMY MAIN LOOP                           |
|                                                                     |
|   +----------+    +----------+    +----------+    +----------+      |
|   |  SENSE   |--->|   PLAN   |--->| CONTROL  |--->| ACTUATE  |      |
|   |          |    |          |    |          |    |          |      |
|   | State    |    | Path     |    | PID /    |    | Command  |      |
|   | Estimat. |    | Planner  |    | MPC      |    | Interface|      |
|   +----------+    +----------+    +----------+    +----------+      |
|        |               |               |               |            |
|        v               v               v               v            |
|   +----------+    +----------+    +----------+    +----------+      |
|   | Position |    | Waypoint |    | Cascaded |    | Velocity |      |
|   | Velocity |    | Spline   |    | Position |    | Commands |      |
|   | Orient.  |    | Racing   |    | Velocity |    | Thrust   |      |
|   | Gate Seq |    | Line     |    | Attitude |    | Rates    |      |
|   +----------+    +----------+    +----------+    +----------+      |
+=====================================================================+
```

### Detailed Data Flow

```
                    +-------------------------+
                    |   COMPETITION PLATFORM   |
                    |   (DCL Simulator)        |
                    +-----------+-------------+
                                | Raw State Data
                                v
                    +-------------------------+
                    |   COMPETITION ADAPTER    |  <-- Thin translation layer
                    |   (competition_adapter)  |      Swap this for the real API
                    +-----------+-------------+
                                | DroneState + RaceCourse
                                v
              +--------------------------------------+
              |         AUTONOMY STACK                |
              |                                       |
              |  +--------------------------------+   |
              |  |     RACE MANAGER               |   |
              |  |  - Gate sequence tracking       |   |
              |  |  - Stall detection              |   |
              |  |  - Mode management              |   |
              |  |    (racing/recovery/done)       |   |
              |  +---------------+----------------+   |
              |                  |                     |
              |  +---------------v----------------+   |
              |  |     PATH PLANNER               |   |
              |  |  - Trajectory generation        |   |
              |  |  - Gate lookahead (N gates)     |   |
              |  |  - Speed profiling              |   |
              |  |  - Curvature analysis           |   |
              |  +---------------+----------------+   |
              |                  | Trajectory + Speed  |
              |                  | Limits              |
              |  +---------------v----------------+   |
              |  |     PURE PURSUIT               |   |
              |  |  - Dynamic lookahead           |   |
              |  |  - Speed-scaled distance       |   |
              |  |  - Target point selection      |   |
              |  +---------------+----------------+   |
              |                  | Target Position     |
              |  +---------------v----------------+   |
              |  |     FLIGHT CONTROLLER          |   |
              |  |                                |   |
              |  |  Position PID --> Vel Cmd       |   |
              |  |  Velocity PID --> Att Cmd       |   |
              |  |  Attitude PID --> Rate Cmd      |   |
              |  |  Thrust Calc --> Throttle       |   |
              |  +---------------+----------------+   |
              |                  | ControlCommand      |
              +------------------+--------------------+
                                 |
                                 v
                    +-------------------------+
                    |   SIMULATOR / PLATFORM   |
                    +-------------------------+
```

### Feedback and Recovery Architecture

```
Normal Operation:
  State --> Plan --> Control --> Execute --> Gate Pass --> Replan
    ^                                             |
    +---------------------------------------------+

Stall Recovery (speed < 1 m/s for extended period):
  Detect Stall --> Switch to Waypoint Planner --> Resume --> Restore Racing Planner

Tilt Recovery (roll/pitch > 30 deg):
  Detect Tilt --> Level Attitude --> Hold Position --> Resume Racing

Gate Miss Recovery (future):
  Detect Miss --> Compute Return Path --> Reattempt Gate --> Continue Sequence
```

---

## Module Breakdown

```
ai-grand-prix/
|
|-- core/                          # Central domain models and brain
|   |-- __init__.py
|   |-- drone_state.py             # Vec3, Quaternion, DroneState, DronePhysics
|   |-- racecourse.py              # Gate, RaceCourse, course factories
|   +-- autonomy.py                # AutonomyStack -- the main loop
|
|-- planning/                      # Trajectory generation
|   |-- __init__.py
|   +-- trajectory.py              # WaypointPlanner, SplinePlanner, RacingLinePlanner
|
|-- control/                       # Flight controllers
|   |-- __init__.py
|   +-- flight_controller.py       # Cascaded PID with anti-windup
|
|-- simulation/                    # Local testing environment
|   |-- __init__.py
|   +-- simulator.py               # 6DOF physics sim with wind, noise, telemetry
|
|-- stress_tests/                  # Automated reliability suite
|   |-- __init__.py
|   +-- run_all.py                 # 30-test comprehensive stress suite
|
|-- config/                        # Tuning parameters and course definitions
|   +-- __init__.py
|
|-- utils/                         # Math helpers, logging, telemetry export
|   +-- __init__.py
|
|-- tests/                         # Unit tests
|   +-- __init__.py
|
|-- run_race.py                    # Quick-launch race runner with CLI args
|-- README.md                      # This file
|-- .gitignore
+-- requirements.txt               # Dependencies
```

### Module Responsibilities

| Module | Purpose | Key Classes |
|--------|---------|-------------|
| `core/drone_state` | Represents drone kinematics, orientation, and physics constants | `Vec3`, `Quaternion`, `DroneState`, `DronePhysics` |
| `core/racecourse` | Gate geometry, passage detection, course generation | `Gate`, `RaceCourse`, factory functions |
| `core/autonomy` | Top-level autonomy loop — connects planning and control | `AutonomyStack`, `AutonomyConfig` |
| `planning/trajectory` | Three trajectory planners with speed profiling | `WaypointPlanner`, `SplinePlanner`, `RacingLinePlanner` |
| `control/flight_controller` | Cascaded PID with position, velocity, and attitude loops | `FlightController`, `PIDController`, `ControlCommand` |
| `simulation/simulator` | Local physics simulation with configurable disturbances | `Simulator`, `SimConfig`, `SimResult`, `SimTelemetry` |
| `stress_tests/run_all` | 30-test automated suite covering all edge cases | 6 test categories, `StressTestResult` |

---

## Technical Deep Dive

### Path Planning — Three Strategies

The planner is the core differentiator. We implement three strategies, each with distinct trade-offs:

```
STRATEGY COMPARISON
===============================================================

  Waypoint Planner          Spline Planner          Racing Line Planner
  -----------------         ---------------         --------------------
  Gate-to-gate direct       Cubic B-spline          Curvature-optimised
  Linear interpolation      C2 smooth curves        Corner-cutting
  Most reliable             Smooth velocity         Fastest theoretical
  Slowest overall           Good balance            Competition target

       G1------G2              G1~~~~G2              G1\    /G2
       |       |              /      \                 \--/
       |       |             /        \                /--\
       G4------G3           G4~~~~~~~~G3             G4/    \G3

  Reliability: *****       Reliability: ****        Reliability: ****
  Speed:       **           Speed:       ****       Speed:       *****
```

**Racing Line Planner** (competition default):
- Computes optimal crossing points within each gate boundary
- Shifts crossing positions toward the inside of turns (corner cutting)
- Uses Menger curvature estimation for dynamic speed profiling
- Higher curvature = lower speed limit, ensuring stability through tight sections
- Parametric B-spline fitting for C2 continuous trajectories

### Flight Controller — Cascaded PID

The controller mirrors real flight controller architectures (PX4, ArduPilot, BetaFlight):

```
CASCADED CONTROL ARCHITECTURE
==============================================================

  Target Position              Current Position
       |                            |
       v                            v
  +---------------------------------------------+
  |         POSITION LOOP (Outer)                |
  |   Kp=4.0  Ki=0.2  Kd=2.0                   |
  |   Error --> Desired Velocity                 |
  +---------------------+------------------------+
                        | Velocity Setpoint
                        v
  +---------------------------------------------+
  |         VELOCITY LOOP (Middle)               |
  |   Kp=5.0  Ki=0.8  Kd=0.8                   |
  |   Error --> Desired Acceleration             |
  |   --> Desired Roll/Pitch Angles              |
  +---------------------+------------------------+
                        | Attitude Setpoint
                        v
  +---------------------------------------------+
  |         ATTITUDE LOOP (Inner)                |
  |   Kp=6.0  Ki=0.0  Kd=0.5                   |
  |   Error --> Rate Commands                    |
  |   + Yaw tracking (face target)               |
  +---------------------+------------------------+
                        | Rate + Thrust Commands
                        v
  +---------------------------------------------+
  |         MOTOR MIXER / OUTPUT                 |
  |   Thrust + Roll/Pitch/Yaw Rates              |
  +---------------------------------------------+
```

**Key features:**
- Anti-windup on all integral terms (clamped accumulators)
- Configurable output limits per axis
- Automatic yaw-to-target tracking
- Tilt limiting (max 45 deg roll/pitch) for safety
- Velocity clamping with independent XY and Z limits

### Simulator — Dual Physics Modes

```
MODE 1: VELOCITY COMMAND (Default)               MODE 2: FULL 6DOF PHYSICS
-------------------------------------            --------------------------
First-order velocity response                    Rigid body dynamics
tau = 0.08s time constant                        Thrust -> body frame rotation
Wind as velocity perturbation                    Gravity + drag + wind forces
Orientation tracks velocity vector               Motor response lag (tau=0.05s)
                                                 Semi-implicit Euler integration
Best for: algorithm development,                 Best for: final validation,
rapid iteration, competition prep                physics fidelity testing
```

### Gate Passage Detection

Gate passage uses a robust plane-crossing algorithm:

```
1. Compute signed distance from drone to gate plane (prev and current frame)
2. Check sign change: negative -> positive (correct crossing direction)
3. Interpolate exact crossing point on the gate plane
4. Project crossing point onto gate coordinate frame
5. Verify within gate bounds + margin (0.3m tolerance)

    ------- Gate Plane -------
    |                         |
    |    +-----------------+  |
    |    |   Valid Zone    |  |   <-- Gate boundary + margin
    |    |       x         |  |   <-- Crossing point (interpolated)
    |    +-----------------+  |
    |                         |
    ---------------------------

    Drone path:  o----------x----------o
                prev      plane      curr
                (d<0)                (d>0)
```

---

## Performance Benchmarks

### Stress Test Results (v0.2.0)

```
+====================================================+
|       STRESS TEST SUITE -- 30/30 PASSING            |
+====================================================+
|                                                      |
|  TEST 1: All Course Types (9 combinations)           |
|  |-- Oval    x 3 planners    PASS PASS PASS         |
|  |-- Sprint  x 3 planners    PASS PASS PASS         |
|  +-- Figure8 x 3 planners    PASS PASS PASS         |
|                                                      |
|  TEST 2: Wind Conditions                             |
|  |-- No wind                 PASS  23.0s             |
|  |-- Light (2 m/s)           PASS  23.1s             |
|  |-- Moderate (5 m/s)        PASS  23.2s             |
|  +-- Heavy (8 m/s + gusts)   PASS  22.7s             |
|                                                      |
|  TEST 3: Sensor Noise                                |
|  |-- Clean                   PASS  23.0s             |
|  |-- Light (0.05m std)       PASS  23.0s             |
|  |-- Moderate (0.15m std)    PASS  23.0s             |
|  +-- Heavy (0.30m std)       PASS  23.0s             |
|                                                      |
|  TEST 4: Gate Size Difficulty                        |
|  |-- Large (5.0m)            PASS  23.0s             |
|  |-- Standard (3.0m)         PASS  23.0s             |
|  |-- Small (2.0m)            PASS  26.7s             |
|  +-- Tiny (1.5m)             PASS  26.7s             |
|                                                      |
|  TEST 5: Consistency (5x repeated)                   |
|  |-- Mean: 24.51s  Std: 0.00s                       |
|  +-- All 5 runs              PASS PASS PASS x5       |
|                                                      |
|  TEST 6: Speed Profiles                              |
|  |-- Conservative            PASS  17.6s             |
|  |-- Moderate                PASS  17.0s             |
|  |-- Aggressive              PASS  16.0s             |
|  +-- Maximum                 PASS  16.2s             |
|                                                      |
+====================================================+
```

### Key Metrics

| Metric | Value |
|--------|-------|
| Gate completion rate | 100% (all conditions) |
| Max speed achieved | 12.6 m/s |
| Average race speed | ~10 m/s |
| Consistency (std dev) | 0.00s across 5 runs |
| Smallest gate passed | 1.5m x 1.5m |
| Max wind survived | 8 m/s + 5 m/s gusts |
| Sim-to-real ratio | ~15x real-time |

---

## Development Roadmap

### Phase 1: Foundation (Complete)

```
[x] Core drone state representation (Vec3, Quaternion, 6DOF state)
[x] Gate/racecourse models with passage detection
[x] Three path planners (Waypoint, Spline, Racing Line)
[x] Cascaded PID flight controller
[x] Local physics simulator (velocity + full 6DOF modes)
[x] Wind, noise, and gate size stress testing
[x] Stall detection and recovery
[x] CLI race runner with configurable parameters
[x] 30/30 stress test suite passing
```

### Phase 2: Speed Optimisation (In Progress)

```
[ ] Model Predictive Control (MPC) -- replace inner PID loops for
    optimal trajectory tracking at high speed. MPC can look ahead
    along the trajectory and pre-compensate for upcoming turns.

[ ] Minimum-Time Trajectory Optimisation -- formulate as an optimal
    control problem: minimise lap time subject to gate constraints,
    velocity limits, and thrust bounds. Solve offline then track online.

[ ] Adaptive Speed Profiling -- dynamically adjust cruise speed based
    on upcoming course geometry. Straight sections -> max speed,
    tight sequences -> pre-computed braking profiles.

[ ] Gate Approach Optimisation -- compute optimal entry/exit vectors
    for each gate considering the full sequence (not just next gate).
    Global optimisation over the entire course.

[ ] Motor Saturation Awareness -- controller knows when it is at
    actuator limits and degrades gracefully instead of integrator
    windup.
```

### Phase 3: Robustness and Intelligence

```
[ ] Extended Kalman Filter (EKF) -- fuse noisy position, velocity, and
    IMU data for robust state estimation. Critical if the competition
    introduces sensor degradation or latency.

[ ] Gate Miss Recovery -- if a gate is missed, compute optimal return
    path and reattempt. Currently the stack assumes sequential success.

[ ] Adaptive PID Tuning -- online gain scheduling based on flight
    regime (hovering vs high-speed cruise vs tight turns).

[ ] Obstacle Awareness -- if the competition introduces obstacles or
    no-fly zones between gates, integrate avoidance into the planner.

[ ] Multi-Lap Strategy -- optimise across multiple laps: aggressive
    first lap for position, then consistent pace management.

[ ] Sensor Fusion Pipeline -- abstract sensor inputs to support
    different competition data formats without changing the core stack.
```

### Phase 4: Competition Integration

```
[ ] Competition Adapter -- thin I/O translation layer between DCL
    platform and our autonomy stack. Will be built when interface
    specs are published.

[ ] Platform-Specific Calibration -- tune physics model to match
    DCL simulator dynamics. Identify discrepancies and compensate.

[ ] Submission Pipeline -- automated build, validate, package, and
    submit workflow. Ensure code runs cleanly on competition hardware.

[ ] Performance Profiling -- ensure the autonomy loop runs within
    the competition's real-time constraints (likely <10ms per tick).

[ ] Anti-Cheat Compliance -- ensure code passes DCL's monitoring
    requirements. No external calls, no human-in-the-loop, pure autonomy.
```

### Phase 5: Competitive Edge

```
[ ] Reinforcement Learning Policy -- train a neural network policy
    via PPO/SAC in simulation, then deploy as the controller. RL can
    discover flight strategies that are difficult to hand-engineer.

[ ] Course Analysis and Pre-Computation -- given a course definition,
    run offline optimisation to compute the globally optimal racing
    line before the run begins. Execute the pre-computed plan online.

[ ] Ensemble Planning -- run multiple planners in parallel, select
    the best trajectory based on predicted completion time and risk.

[ ] Monte Carlo Stress Testing -- randomised course generation with
    statistical analysis across thousands of runs to find failure modes.

[ ] Telemetry Dashboard -- real-time visualisation of flight path,
    speed profile, gate passages, controller errors, and system health
    for rapid debugging and tuning.
```

### Timeline Targets

```
        Feb 2026                    Mar 2026                    Apr 2026
     -----------------------------------------------------------------------
     | Phase 1    |    Phase 2    |    Phase 3    |  Phase 4  |    P5     |
     | Foundation |    Speed      |  Robustness   |  Comp.    |   Edge    |
     | COMPLETE   |    Optim.     |  & Intel.     |  Integ.   |           |
     -----------------------------------------------------------------------
                   ^                                ^
                   |                                |
             Interface specs                   Round 1
             expected (est.)                   deadline
```

---

## Quick Start

### Prerequisites

- Python 3.10+
- pip

### Installation

```bash
git clone https://github.com/YOUR_USERNAME/ai-grand-prix.git
cd ai-grand-prix
pip install numpy scipy matplotlib
```

### Run Your First Race

```bash
# Default: oval course, racing line planner
python run_race.py

# Sprint course with verbose output
python run_race.py --course sprint --verbose

# Figure-8 with wind enabled
python run_race.py --course figure8 --wind

# Full stress test suite
python -m stress_tests.run_all
```

---

## Usage Guide

### Race Runner CLI

```bash
python run_race.py [OPTIONS]

Options:
  --course {oval,sprint,figure8}     Course type (default: oval)
  --planner {waypoint,spline,racing} Planner strategy (default: racing)
  --gates N                          Number of gates (default: 10)
  --gate-size F                      Gate dimensions in meters (default: 3.0)
  --cruise-speed F                   Maximum cruise speed m/s (default: 18.0)
  --gate-speed F                     Speed limit near gates m/s (default: 10.0)
  --time-limit F                     Max race duration seconds (default: 120.0)
  --wind                             Enable wind disturbances
  --noise F                          Position sensor noise std in meters
  --verbose, -v                      Print tick-by-tick telemetry
```

### Stress Test Suite

```bash
# Run all 30 tests
python -m stress_tests.run_all

# Verbose mode (tick-by-tick for each test)
python -m stress_tests.run_all --verbose
```

### Using the Autonomy Stack Programmatically

```python
from core.racecourse import get_course
from core.autonomy import AutonomyConfig, create_autonomy_fn
from simulation.simulator import SimConfig, run_simulation

# Configure
course = get_course("oval", num_gates=10, gate_size=3.0)
config = AutonomyConfig()
config.planner_type = "racing"
config.cruise_speed = 18.0

# Run
autonomy = create_autonomy_fn(config)
result = run_simulation(course, autonomy, SimConfig())

# Analyse
print(f"Gates: {result.gates_passed}/{result.total_gates}")
print(f"Time: {result.race_time:.2f}s")
print(f"Max speed: {result.telemetry.max_speed():.1f} m/s")
```

### Custom Course Definition

```python
from core.drone_state import Vec3
from core.racecourse import Gate, RaceCourse

course = RaceCourse(
    name="Custom Course",
    gates=[
        Gate(index=0, center=Vec3(10, 0, -10), normal=Vec3(1, 0, 0), width=3.0, height=3.0),
        Gate(index=1, center=Vec3(20, 5, -12), normal=Vec3(1, 0.2, 0), width=3.0, height=3.0),
        # ... add more gates
    ],
    start_position=Vec3(0, 0, -10),
    time_limit=60.0,
)
```

---

## Design Philosophy

### 1. Reliability Over Speed

> *"A fast drone that misses a gate scores zero. A slow drone that completes all gates wins."*

The scoring system prioritises gate completion. Our architecture enforces this: every speed optimisation is gated behind a reliability check. The racing line planner reduces speed through high-curvature sections. The stall detector falls back to the simplest planner if the drone gets stuck.

### 2. Modularity at Every Layer

The competition interface is unknown until they publish specs. Our defence:

```
Competition Platform  <-->  Adapter (1 file)  <-->  Autonomy Stack (unchanged)
```

When the DCL platform SDK drops, we write one adapter file. The entire planning, control, and intelligence stack remains untouched. This also means we can swap planners, controllers, and state estimators independently.

### 3. Test in Conditions Worse Than Race Day

Every module is validated against conditions more extreme than what we expect in competition. If the stack survives 8 m/s wind with 5 m/s gusts, 0.3m sensor noise, and 1.5m gates simultaneously, it will handle race-day conditions.

### 4. Deterministic and Reproducible

Every simulation run is logged with full telemetry. Same inputs produce same outputs (0.00s standard deviation across repeated runs). This makes debugging precise and regression testing reliable.

### 5. Speed is Earned Through Intelligence

Rather than cranking up velocity limits and hoping, speed comes from:
- Smarter trajectories (racing lines that reduce total path length)
- Better control (tighter tracking = less wasted motion)
- Predictive speed profiling (brake before the turn, not in it)
- Global course optimisation (look at all gates, not just the next one)

---

## Competition Adapter Strategy

When Anduril/DCL publish the interface specifications, the integration follows this pattern:

```python
# competition_adapter.py -- THE ONLY FILE THAT CHANGES

from core.autonomy import AutonomyStack, AutonomyConfig
from core.drone_state import DroneState, Vec3, Quaternion
from core.racecourse import RaceCourse, Gate

class CompetitionAdapter:
    """Translates between DCL platform API and our autonomy stack."""

    def __init__(self):
        self.stack = AutonomyStack(AutonomyConfig())

    def on_state_update(self, platform_state: dict) -> dict:
        """
        Called by DCL platform each tick.
        Translate their state -> our DroneState.
        Run autonomy.
        Translate our ControlCommand -> their expected format.
        """
        # 1. Translate input
        state = self._translate_state(platform_state)
        course = self._translate_course(platform_state)

        # 2. Run autonomy (unchanged)
        command = self.stack.update(state, course)

        # 3. Translate output
        return self._translate_command(command)

    def _translate_state(self, raw: dict) -> DroneState:
        # Map platform fields to DroneState
        ...

    def _translate_course(self, raw: dict) -> RaceCourse:
        # Map platform gate definitions to RaceCourse
        ...

    def _translate_command(self, cmd) -> dict:
        # Map ControlCommand to platform expected output format
        ...
```

---

## Contributing

This is a competitive entry. Contributions are welcome from team members only.

**Development workflow:**
1. Branch from `main` for each feature
2. Run `python -m stress_tests.run_all` before merging — all 30 must pass
3. No merge without 100% gate completion on all course types
4. Performance regressions require justification

**Code standards:**
- Type hints on all function signatures
- Docstrings on all public classes and methods
- No external network calls in the autonomy stack
- All tuning parameters exposed through config objects, never hardcoded

---

<div align="center">

**Built for speed. Tested for survival. Ready for competition.**

*AI Grand Prix — Anduril Industries x Drone Champions League*

</div>