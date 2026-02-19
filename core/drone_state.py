"""
Core drone state representation and physics model.

The DroneState is the single source of truth for the drone's current
condition. All modules read from and write to this structure.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Optional
import time


@dataclass
class Vec3:
    """3D vector with utility methods."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z], dtype=np.float64)

    @classmethod
    def from_array(cls, arr: np.ndarray) -> 'Vec3':
        return cls(x=float(arr[0]), y=float(arr[1]), z=float(arr[2]))

    def magnitude(self) -> float:
        return float(np.linalg.norm(self.to_array()))

    def normalized(self) -> 'Vec3':
        mag = self.magnitude()
        if mag < 1e-10:
            return Vec3(0, 0, 0)
        arr = self.to_array() / mag
        return Vec3.from_array(arr)

    def distance_to(self, other: 'Vec3') -> float:
        return float(np.linalg.norm(self.to_array() - other.to_array()))

    def __add__(self, other: 'Vec3') -> 'Vec3':
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: 'Vec3') -> 'Vec3':
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> 'Vec3':
        return Vec3(self.x * scalar, self.y * scalar, self.z * scalar)

    def __repr__(self) -> str:
        return f"Vec3({self.x:.3f}, {self.y:.3f}, {self.z:.3f})"

    def dot(self, other: 'Vec3') -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other: 'Vec3') -> 'Vec3':
        return Vec3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x
        )


@dataclass
class Quaternion:
    """Quaternion for rotation representation (w, x, y, z convention)."""
    w: float = 1.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def to_array(self) -> np.ndarray:
        return np.array([self.w, self.x, self.y, self.z], dtype=np.float64)

    @classmethod
    def from_euler(cls, roll: float, pitch: float, yaw: float) -> 'Quaternion':
        """Create from Euler angles (radians), ZYX convention."""
        cr, sr = np.cos(roll / 2), np.sin(roll / 2)
        cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
        cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
        return cls(
            w=cr * cp * cy + sr * sp * sy,
            x=sr * cp * cy - cr * sp * sy,
            y=cr * sp * cy + sr * cp * sy,
            z=cr * cp * sy - sr * sp * cy
        )

    def to_euler(self) -> tuple:
        """Returns (roll, pitch, yaw) in radians."""
        sinr_cosp = 2 * (self.w * self.x + self.y * self.z)
        cosr_cosp = 1 - 2 * (self.x * self.x + self.y * self.y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (self.w * self.y - self.z * self.x)
        sinp = np.clip(sinp, -1.0, 1.0)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (self.w * self.z + self.x * self.y)
        cosy_cosp = 1 - 2 * (self.y * self.y + self.z * self.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return (roll, pitch, yaw)

    def forward_vector(self) -> Vec3:
        """Get the forward direction vector from this orientation."""
        # Forward is +X in body frame
        x = 1 - 2 * (self.y**2 + self.z**2)
        y = 2 * (self.x * self.y + self.w * self.z)
        z = 2 * (self.x * self.z - self.w * self.y)
        return Vec3(x, y, z).normalized()

    def normalized(self) -> 'Quaternion':
        arr = self.to_array()
        n = np.linalg.norm(arr)
        if n < 1e-10:
            return Quaternion()
        arr /= n
        return Quaternion(w=arr[0], x=arr[1], y=arr[2], z=arr[3])

    def __repr__(self) -> str:
        return f"Quat(w={self.w:.3f}, x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f})"


@dataclass
class DroneState:
    """Complete drone state at a single instant."""
    # Kinematics
    position: Vec3 = field(default_factory=Vec3)
    velocity: Vec3 = field(default_factory=Vec3)
    acceleration: Vec3 = field(default_factory=Vec3)
    orientation: Quaternion = field(default_factory=Quaternion)
    angular_velocity: Vec3 = field(default_factory=Vec3)

    # Race progress
    current_gate_index: int = 0
    gates_passed: int = 0
    total_gates: int = 0
    race_time: float = 0.0
    race_finished: bool = False

    # Health
    battery_pct: float = 100.0
    motor_health: list = field(default_factory=lambda: [1.0, 1.0, 1.0, 1.0])

    # Telemetry timestamp
    timestamp: float = field(default_factory=time.time)

    def speed(self) -> float:
        return self.velocity.magnitude()

    def altitude(self) -> float:
        return -self.position.z  # NED convention: Z down is negative altitude

    def heading(self) -> float:
        """Yaw angle in radians."""
        return self.orientation.to_euler()[2]

    def progress_pct(self) -> float:
        if self.total_gates == 0:
            return 0.0
        return (self.gates_passed / self.total_gates) * 100.0

    def clone(self) -> 'DroneState':
        """Deep copy the state."""
        import copy
        return copy.deepcopy(self)


# ─── Physics Constants ─────────────────────────────────────────────
class DronePhysics:
    """Realistic quadrotor physical parameters."""
    MASS_KG = 1.5
    GRAVITY = 9.81
    DRAG_COEFF = 0.25  # aerodynamic drag coefficient

    # Motor limits
    MAX_THRUST_N = 4 * 9.81  # ~4x weight for agility
    MAX_ROLL_RATE = np.radians(720)   # deg/s → rad/s
    MAX_PITCH_RATE = np.radians(720)
    MAX_YAW_RATE = np.radians(360)
    MAX_VELOCITY = 20.0  # m/s

    # Moment of inertia (simplified)
    I_XX = 0.015  # kg*m^2
    I_YY = 0.015
    I_ZZ = 0.025

    # Motor response time constant
    MOTOR_TAU = 0.05  # seconds

    @classmethod
    def thrust_to_acceleration(cls, thrust: float) -> float:
        """Net vertical acceleration from thrust force."""
        return (thrust / cls.MASS_KG) - cls.GRAVITY

    @classmethod
    def drag_force(cls, velocity: Vec3) -> Vec3:
        """Quadratic drag opposing velocity."""
        v = velocity.to_array()
        speed = np.linalg.norm(v)
        if speed < 1e-6:
            return Vec3()
        drag = -cls.DRAG_COEFF * speed * v
        return Vec3.from_array(drag)