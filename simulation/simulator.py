"""
Local Physics Simulator — Tests autonomy stack without external dependencies.

Simulates:
- Quadrotor rigid body dynamics (6DOF)
- Aerodynamic drag
- Motor response delay
- Gate passage detection
- Wind disturbances (configurable)
- Sensor noise (configurable)
"""

import numpy as np
import time as time_module
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Callable
from core.drone_state import Vec3, Quaternion, DroneState, DronePhysics
from core.racecourse import Gate, RaceCourse
from control.flight_controller import ControlCommand


@dataclass
class SimConfig:
    """Simulator configuration."""
    dt: float = 0.01                  # Physics timestep (100Hz)
    real_time: bool = False           # Run in real-time or as fast as possible
    max_sim_time: float = 120.0       # Max simulation duration

    # Disturbances
    wind_enabled: bool = False
    wind_mean: Vec3 = field(default_factory=lambda: Vec3(0, 0, 0))
    wind_gust_magnitude: float = 0.0  # m/s
    wind_gust_frequency: float = 0.5  # Hz

    # Sensor noise
    position_noise_std: float = 0.0   # meters
    velocity_noise_std: float = 0.0   # m/s
    orientation_noise_std: float = 0.0 # radians

    # Failure modes
    motor_degradation: List[float] = field(default_factory=lambda: [1.0, 1.0, 1.0, 1.0])


@dataclass
class SimTelemetry:
    """Recorded telemetry for analysis."""
    timestamps: List[float] = field(default_factory=list)
    positions: List[Vec3] = field(default_factory=list)
    velocities: List[Vec3] = field(default_factory=list)
    speeds: List[float] = field(default_factory=list)
    gate_pass_times: Dict[int, float] = field(default_factory=dict)
    commands: List[ControlCommand] = field(default_factory=list)
    errors: List[float] = field(default_factory=list)  # Position error to target

    def max_speed(self) -> float:
        return max(self.speeds) if self.speeds else 0.0

    def avg_speed(self) -> float:
        return float(np.mean(self.speeds)) if self.speeds else 0.0

    def total_distance(self) -> float:
        dist = 0.0
        for i in range(1, len(self.positions)):
            dist += self.positions[i - 1].distance_to(self.positions[i])
        return dist


@dataclass
class SimResult:
    """Complete simulation result."""
    success: bool = False
    gates_passed: int = 0
    total_gates: int = 0
    race_time: float = 0.0
    timed_out: bool = False
    crashed: bool = False
    crash_reason: str = ""
    telemetry: SimTelemetry = field(default_factory=SimTelemetry)

    def summary(self) -> str:
        status = "SUCCESS" if self.success else "FAILED"
        if self.crashed:
            status = f"CRASHED ({self.crash_reason})"
        if self.timed_out:
            status = "TIMED OUT"

        return (
            f"[{status}] Gates: {self.gates_passed}/{self.total_gates} | "
            f"Time: {self.race_time:.2f}s | "
            f"Max Speed: {self.telemetry.max_speed():.1f} m/s | "
            f"Avg Speed: {self.telemetry.avg_speed():.1f} m/s | "
            f"Distance: {self.telemetry.total_distance():.1f}m"
        )


class Simulator:
    """6DOF quadrotor physics simulator."""

    def __init__(self, config: Optional[SimConfig] = None):
        self.config = config or SimConfig()
        self.state = DroneState()
        self.telemetry = SimTelemetry()
        self.sim_time = 0.0

    def reset(self, course: RaceCourse):
        """Initialize simulation with a racecourse."""
        self.state = DroneState(
            position=course.start_position.clone() if hasattr(course.start_position, 'clone')
                     else Vec3(course.start_position.x, course.start_position.y, course.start_position.z),
            orientation=course.start_orientation,
            total_gates=course.num_gates,
        )
        self.telemetry = SimTelemetry()
        self.sim_time = 0.0
        course.reset()

    def step(self, command: ControlCommand, course: RaceCourse) -> DroneState:
        """
        Advance simulation by one timestep.
        Returns the (possibly noisy) observed state.
        """
        dt = self.config.dt
        prev_pos = Vec3(self.state.position.x, self.state.position.y, self.state.position.z)

        # ── Velocity command mode (higher-level, more realistic for competition) ──
        if command.velocity_cmd is not None:
            # First-order velocity response with drag
            tau = 0.08  # response time constant (snappy)
            alpha = dt / (tau + dt)
            
            target_vel = command.velocity_cmd
            new_vel = Vec3(
                self.state.velocity.x + alpha * (target_vel.x - self.state.velocity.x),
                self.state.velocity.y + alpha * (target_vel.y - self.state.velocity.y),
                self.state.velocity.z + alpha * (target_vel.z - self.state.velocity.z),
            )
            
            # Add wind disturbance
            wind = self._compute_wind()
            wind_vel = wind * (1.0 / DronePhysics.MASS_KG) * dt
            new_vel = new_vel + wind_vel
            
            # Clamp velocity
            speed = new_vel.magnitude()
            if speed > DronePhysics.MAX_VELOCITY:
                new_vel = new_vel * (DronePhysics.MAX_VELOCITY / speed)
            
            # Update position
            new_pos = Vec3(
                self.state.position.x + new_vel.x * dt,
                self.state.position.y + new_vel.y * dt,
                self.state.position.z + new_vel.z * dt,
            )
            
            # Update orientation to face velocity direction
            speed = new_vel.magnitude()
            if speed > 0.5:
                yaw = np.arctan2(new_vel.y, new_vel.x)
                pitch = np.arctan2(-new_vel.z, np.sqrt(new_vel.x**2 + new_vel.y**2))
                self.state.orientation = Quaternion.from_euler(0, pitch * 0.3, yaw)
            
            self.state.position = new_pos
            self.state.velocity = new_vel
            self.state.acceleration = Vec3()
            self.sim_time += dt
            self.state.race_time = self.sim_time
        else:
            # ── Full physics mode ──
            self._step_full_physics(command, dt)

        # ── Check gate passages ──
        if self.state.current_gate_index < course.num_gates:
            current_gate = course.gates[self.state.current_gate_index]
            if current_gate.check_passage(prev_pos, self.state.position):
                current_gate.passed = True
                current_gate.pass_time = self.sim_time
                self.state.gates_passed += 1
                self.state.current_gate_index += 1
                self.telemetry.gate_pass_times[current_gate.index] = self.sim_time

                if self.state.current_gate_index >= course.num_gates:
                    self.state.race_finished = True

        # ── Record telemetry ──
        speed = self.state.velocity.magnitude()
        self.telemetry.timestamps.append(self.sim_time)
        self.telemetry.positions.append(Vec3(self.state.position.x, self.state.position.y, self.state.position.z))
        self.telemetry.velocities.append(Vec3(self.state.velocity.x, self.state.velocity.y, self.state.velocity.z))
        self.telemetry.speeds.append(speed)
        self.telemetry.commands.append(command)

        # Return observed state (with optional noise)
        return self._apply_sensor_noise(self.state.clone())

    def _step_full_physics(self, command: ControlCommand, dt: float):
        # Thrust (in body Z-down frame, so thrust is negative Z in body)
        thrust_force = command.thrust * DronePhysics.MAX_THRUST_N

        # Get orientation for thrust direction
        roll, pitch, yaw = self.state.orientation.to_euler()

        # In NED frame, thrust acts primarily in -Z direction (upward)
        # When tilted, it produces horizontal acceleration
        thrust_world = Vec3(
            -thrust_force * np.sin(pitch),
            thrust_force * np.cos(pitch) * np.sin(roll),
            -thrust_force * np.cos(pitch) * np.cos(roll)
        )

        # Gravity (NED: positive Z is down)
        gravity = Vec3(0, 0, DronePhysics.MASS_KG * DronePhysics.GRAVITY)

        # Drag
        drag = DronePhysics.drag_force(self.state.velocity)

        # Wind disturbance
        wind = self._compute_wind()

        # Total acceleration
        total_force = thrust_world + gravity + drag + wind
        acc = total_force * (1.0 / DronePhysics.MASS_KG)

        # ── Integrate kinematics (semi-implicit Euler) ──
        # Update velocity first
        new_vel = Vec3(
            self.state.velocity.x + acc.x * dt,
            self.state.velocity.y + acc.y * dt,
            self.state.velocity.z + acc.z * dt,
        )

        # Clamp velocity
        speed = new_vel.magnitude()
        if speed > DronePhysics.MAX_VELOCITY:
            scale = DronePhysics.MAX_VELOCITY / speed
            new_vel = new_vel * scale

        # Update position
        new_pos = Vec3(
            self.state.position.x + new_vel.x * dt,
            self.state.position.y + new_vel.y * dt,
            self.state.position.z + new_vel.z * dt,
        )

        # ── Update orientation ──
        # Apply rate commands with motor response lag
        tau = DronePhysics.MOTOR_TAU
        alpha = dt / (tau + dt)  # First-order filter

        actual_roll_rate = self.state.angular_velocity.x + alpha * (command.roll_rate - self.state.angular_velocity.x)
        actual_pitch_rate = self.state.angular_velocity.y + alpha * (command.pitch_rate - self.state.angular_velocity.y)
        actual_yaw_rate = self.state.angular_velocity.z + alpha * (command.yaw_rate - self.state.angular_velocity.z)

        new_roll = roll + actual_roll_rate * dt
        new_pitch = pitch + actual_pitch_rate * dt
        new_yaw = yaw + actual_yaw_rate * dt

        new_orientation = Quaternion.from_euler(new_roll, new_pitch, new_yaw).normalized()

        # ── Update state ──
        self.state.position = new_pos
        self.state.velocity = new_vel
        self.state.acceleration = acc
        self.state.orientation = new_orientation
        self.state.angular_velocity = Vec3(actual_roll_rate, actual_pitch_rate, actual_yaw_rate)
        self.sim_time += dt
        self.state.race_time = self.sim_time

    def check_crash(self) -> tuple:
        """Check for crash conditions. Returns (crashed: bool, reason: str)."""
        # Ground collision (NED: Z > 0 means below ground)
        if self.state.position.z > 0.5:
            return True, "ground_collision"

        # Too high
        if self.state.position.z < -200:
            return True, "altitude_limit"

        # Excessive tilt
        roll, pitch, _ = self.state.orientation.to_euler()
        if abs(roll) > np.radians(80) or abs(pitch) > np.radians(80):
            return True, "excessive_tilt"

        # Out of bounds (very far from course)
        if self.state.position.magnitude() > 500:
            return True, "out_of_bounds"

        return False, ""

    def _compute_wind(self) -> Vec3:
        """Compute wind disturbance at current time."""
        if not self.config.wind_enabled:
            return Vec3()

        # Mean wind
        wind = self.config.wind_mean.to_array()

        # Add gusts (sinusoidal + random)
        if self.config.wind_gust_magnitude > 0:
            gust = self.config.wind_gust_magnitude * np.sin(
                2 * np.pi * self.config.wind_gust_frequency * self.sim_time
            )
            gust_dir = np.random.randn(3) * 0.3  # Random direction variation
            gust_dir = gust_dir / (np.linalg.norm(gust_dir) + 1e-10)
            wind += gust * gust_dir

        force = wind * DronePhysics.DRAG_COEFF * DronePhysics.MASS_KG
        return Vec3.from_array(force)

    def _apply_sensor_noise(self, state: DroneState) -> DroneState:
        """Add sensor noise to observed state."""
        if self.config.position_noise_std > 0:
            noise = np.random.randn(3) * self.config.position_noise_std
            state.position = Vec3(
                state.position.x + noise[0],
                state.position.y + noise[1],
                state.position.z + noise[2],
            )

        if self.config.velocity_noise_std > 0:
            noise = np.random.randn(3) * self.config.velocity_noise_std
            state.velocity = Vec3(
                state.velocity.x + noise[0],
                state.velocity.y + noise[1],
                state.velocity.z + noise[2],
            )

        return state


def run_simulation(
    course: RaceCourse,
    autonomy_fn: Callable[[DroneState, RaceCourse], ControlCommand],
    config: Optional[SimConfig] = None,
    verbose: bool = True,
) -> SimResult:
    """
    Run a complete race simulation.

    Args:
        course: The racecourse to fly
        autonomy_fn: Function that takes (state, course) and returns a ControlCommand
        config: Simulation configuration
        verbose: Print progress updates

    Returns:
        SimResult with success/failure and telemetry
    """
    config = config or SimConfig()
    sim = Simulator(config)
    sim.reset(course)

    result = SimResult(total_gates=course.num_gates)
    step_count = 0
    print_interval = int(1.0 / config.dt)  # Print every ~1 second of sim time

    while sim.sim_time < config.max_sim_time:
        # Get observed state
        observed_state = sim.state.clone()

        # Run autonomy
        try:
            command = autonomy_fn(observed_state, course)
        except Exception as e:
            result.crashed = True
            result.crash_reason = f"autonomy_exception: {e}"
            break

        # Step simulation
        sim.step(command, course)

        # Check crash
        crashed, reason = sim.check_crash()
        if crashed:
            result.crashed = True
            result.crash_reason = reason
            break

        # Check race complete
        if sim.state.race_finished:
            result.success = True
            break

        # Progress reporting
        step_count += 1
        if verbose and step_count % print_interval == 0:
            print(f"  t={sim.sim_time:.1f}s | "
                  f"Gates: {sim.state.gates_passed}/{course.num_gates} | "
                  f"Speed: {sim.state.speed():.1f} m/s | "
                  f"Alt: {sim.state.altitude():.1f}m")

    # Fill result
    result.gates_passed = sim.state.gates_passed
    result.race_time = sim.sim_time
    result.timed_out = (sim.sim_time >= config.max_sim_time and not sim.state.race_finished)
    result.telemetry = sim.telemetry

    if verbose:
        print(f"\n  {result.summary()}")

    return result