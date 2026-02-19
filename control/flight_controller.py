"""
Flight Controller — Cascaded PID with position, velocity, and attitude loops.

Architecture:
    Position Error → Velocity Command → Attitude Command → Motor Mixing

This is the standard cascade control used in real flight controllers
(PX4, ArduPilot, BetaFlight).
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Tuple, Optional
from core.drone_state import Vec3, DroneState, DronePhysics, Quaternion


@dataclass
class PIDGains:
    """PID gain set for a single axis."""
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0
    i_max: float = 5.0   # Integral windup limit
    output_max: float = float('inf')


@dataclass
class PIDState:
    """Internal state of a PID controller."""
    integral: float = 0.0
    prev_error: float = 0.0
    prev_output: float = 0.0


class PIDController:
    """Single-axis PID controller with anti-windup."""

    def __init__(self, gains: PIDGains):
        self.gains = gains
        self.state = PIDState()

    def update(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0

        # Proportional
        p = self.gains.kp * error

        # Integral with anti-windup
        self.state.integral += error * dt
        self.state.integral = np.clip(
            self.state.integral, -self.gains.i_max, self.gains.i_max
        )
        i = self.gains.ki * self.state.integral

        # Derivative (on error, with filtering)
        if self.state.prev_error is not None:
            d_raw = (error - self.state.prev_error) / dt
        else:
            d_raw = 0.0
        d = self.gains.kd * d_raw

        self.state.prev_error = error

        output = p + i + d
        output = np.clip(output, -self.gains.output_max, self.gains.output_max)
        self.state.prev_output = output

        return float(output)

    def reset(self):
        self.state = PIDState()


@dataclass
class ControlCommand:
    """Output command to the drone's motor mixer / simulation."""
    thrust: float = 0.0        # Total thrust [0, 1] normalized
    roll_rate: float = 0.0     # Desired roll rate (rad/s)
    pitch_rate: float = 0.0    # Desired pitch rate (rad/s)
    yaw_rate: float = 0.0      # Desired yaw rate (rad/s)

    # For simulation: direct velocity commands (optional)
    velocity_cmd: Optional[Vec3] = None

    def to_array(self) -> np.ndarray:
        return np.array([self.thrust, self.roll_rate, self.pitch_rate, self.yaw_rate])


@dataclass
class FlightControllerConfig:
    """Tunable parameters for the flight controller."""
    # Position loop gains
    pos_xy: PIDGains = field(default_factory=lambda: PIDGains(kp=4.0, ki=0.2, kd=2.0, output_max=20.0))
    pos_z: PIDGains = field(default_factory=lambda: PIDGains(kp=5.0, ki=0.3, kd=2.5, output_max=15.0))

    # Velocity loop gains
    vel_xy: PIDGains = field(default_factory=lambda: PIDGains(kp=5.0, ki=0.8, kd=0.8, output_max=20.0))
    vel_z: PIDGains = field(default_factory=lambda: PIDGains(kp=6.0, ki=1.0, kd=0.5, output_max=15.0))

    # Attitude gains
    att_roll: PIDGains = field(default_factory=lambda: PIDGains(kp=6.0, ki=0.0, kd=0.5))
    att_pitch: PIDGains = field(default_factory=lambda: PIDGains(kp=6.0, ki=0.0, kd=0.5))
    att_yaw: PIDGains = field(default_factory=lambda: PIDGains(kp=4.0, ki=0.1, kd=0.3))

    # Limits
    max_tilt_angle: float = np.radians(45)  # Max roll/pitch
    max_velocity: float = 18.0  # m/s
    max_vertical_velocity: float = 5.0

    # Lookahead
    lookahead_distance: float = 5.0  # meters
    position_tolerance: float = 1.0   # meters - "close enough" to waypoint


class FlightController:
    """
    Cascaded PID flight controller.

    Outer loop: Position → desired velocity
    Middle loop: Velocity → desired attitude
    Inner loop: Attitude → rate commands
    """

    def __init__(self, config: Optional[FlightControllerConfig] = None):
        self.config = config or FlightControllerConfig()

        # Position controllers
        self.pos_x_pid = PIDController(self.config.pos_xy)
        self.pos_y_pid = PIDController(self.config.pos_xy)
        self.pos_z_pid = PIDController(self.config.pos_z)

        # Velocity controllers
        self.vel_x_pid = PIDController(self.config.vel_xy)
        self.vel_y_pid = PIDController(self.config.vel_xy)
        self.vel_z_pid = PIDController(self.config.vel_z)

        # Attitude controllers
        self.roll_pid = PIDController(self.config.att_roll)
        self.pitch_pid = PIDController(self.config.att_pitch)
        self.yaw_pid = PIDController(self.config.att_yaw)

    def compute(self, state: DroneState, target_position: Vec3,
                target_yaw: Optional[float] = None,
                speed_limit: float = 15.0,
                dt: float = 0.01) -> ControlCommand:
        """
        Main control loop: given current state and target, compute motor commands.
        """
        # ── Position Loop → Velocity setpoint ──
        pos_err_x = target_position.x - state.position.x
        pos_err_y = target_position.y - state.position.y
        pos_err_z = target_position.z - state.position.z

        vel_cmd_x = self.pos_x_pid.update(pos_err_x, dt)
        vel_cmd_y = self.pos_y_pid.update(pos_err_y, dt)
        vel_cmd_z = self.pos_z_pid.update(pos_err_z, dt)

        # Clamp velocity command to speed limit
        vel_xy = np.array([vel_cmd_x, vel_cmd_y])
        vel_xy_mag = np.linalg.norm(vel_xy)
        actual_limit = min(speed_limit, self.config.max_velocity)
        if vel_xy_mag > actual_limit:
            vel_xy = vel_xy / vel_xy_mag * actual_limit
            vel_cmd_x, vel_cmd_y = vel_xy

        vel_cmd_z = np.clip(vel_cmd_z,
                           -self.config.max_vertical_velocity,
                           self.config.max_vertical_velocity)

        # ── Velocity Loop → Acceleration / Attitude setpoint ──
        vel_err_x = vel_cmd_x - state.velocity.x
        vel_err_y = vel_cmd_y - state.velocity.y
        vel_err_z = vel_cmd_z - state.velocity.z

        acc_cmd_x = self.vel_x_pid.update(vel_err_x, dt)
        acc_cmd_y = self.vel_y_pid.update(vel_err_y, dt)
        acc_cmd_z = self.vel_z_pid.update(vel_err_z, dt)

        # ── Convert desired acceleration to attitude ──
        # For a quadrotor, horizontal acceleration requires tilting
        yaw = state.heading()

        # Rotate acceleration command to body frame
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        # Desired pitch and roll from desired acceleration (small angle approx)
        desired_pitch = np.arctan2(
            acc_cmd_x * cos_yaw + acc_cmd_y * sin_yaw,
            DronePhysics.GRAVITY
        )
        desired_roll = np.arctan2(
            acc_cmd_y * cos_yaw - acc_cmd_x * sin_yaw,
            DronePhysics.GRAVITY
        )

        # Clamp tilt
        desired_pitch = np.clip(desired_pitch,
                               -self.config.max_tilt_angle,
                               self.config.max_tilt_angle)
        desired_roll = np.clip(desired_roll,
                              -self.config.max_tilt_angle,
                              self.config.max_tilt_angle)

        # ── Attitude Loop → Rate commands ──
        current_roll, current_pitch, current_yaw = state.orientation.to_euler()

        roll_err = desired_roll - current_roll
        pitch_err = desired_pitch - current_pitch

        # Yaw: point toward target if no explicit yaw given
        if target_yaw is None:
            dx = target_position.x - state.position.x
            dy = target_position.y - state.position.y
            if abs(dx) > 0.5 or abs(dy) > 0.5:
                target_yaw = np.arctan2(dy, dx)
            else:
                target_yaw = current_yaw

        yaw_err = self._wrap_angle(target_yaw - current_yaw)

        roll_rate = self.roll_pid.update(roll_err, dt)
        pitch_rate = self.pitch_pid.update(pitch_err, dt)
        yaw_rate = self.yaw_pid.update(yaw_err, dt)

        # ── Thrust ──
        # Base thrust to hover + Z correction
        hover_thrust = DronePhysics.MASS_KG * DronePhysics.GRAVITY
        thrust_correction = DronePhysics.MASS_KG * acc_cmd_z
        total_thrust = hover_thrust - thrust_correction  # NED: negative Z is up
        thrust_normalized = np.clip(total_thrust / DronePhysics.MAX_THRUST_N, 0.0, 1.0)

        return ControlCommand(
            thrust=float(thrust_normalized),
            roll_rate=float(np.clip(roll_rate, -DronePhysics.MAX_ROLL_RATE, DronePhysics.MAX_ROLL_RATE)),
            pitch_rate=float(np.clip(pitch_rate, -DronePhysics.MAX_PITCH_RATE, DronePhysics.MAX_PITCH_RATE)),
            yaw_rate=float(np.clip(yaw_rate, -DronePhysics.MAX_YAW_RATE, DronePhysics.MAX_YAW_RATE)),
            velocity_cmd=Vec3(vel_cmd_x, vel_cmd_y, vel_cmd_z),
        )

    def reset(self):
        """Reset all PID states."""
        for pid in [self.pos_x_pid, self.pos_y_pid, self.pos_z_pid,
                    self.vel_x_pid, self.vel_y_pid, self.vel_z_pid,
                    self.roll_pid, self.pitch_pid, self.yaw_pid]:
            pid.reset()

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """Wrap angle to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle