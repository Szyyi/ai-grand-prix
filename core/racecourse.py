"""
Gate and racecourse definitions.

Gates are rectangular regions in 3D space. A gate is 'passed' when the drone
crosses the gate plane from the correct side within the gate boundary.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
from core.drone_state import Vec3, Quaternion


@dataclass
class Gate:
    """A single race gate."""
    index: int
    center: Vec3
    normal: Vec3        # Gate facing direction (drone must cross in this direction)
    width: float = 3.0  # meters
    height: float = 3.0
    passed: bool = False
    pass_time: Optional[float] = None

    def corners(self) -> List[Vec3]:
        """Get the 4 corner positions of the gate."""
        # Build gate coordinate frame
        n = self.normal.to_array()
        n = n / np.linalg.norm(n)

        # Pick an up vector that's not parallel to normal
        up_candidate = np.array([0, 0, 1])
        if abs(np.dot(n, up_candidate)) > 0.99:
            up_candidate = np.array([0, 1, 0])

        right = np.cross(n, up_candidate)
        right = right / np.linalg.norm(right)
        up = np.cross(right, n)
        up = up / np.linalg.norm(up)

        c = self.center.to_array()
        hw, hh = self.width / 2, self.height / 2

        return [
            Vec3.from_array(c - right * hw - up * hh),
            Vec3.from_array(c + right * hw - up * hh),
            Vec3.from_array(c + right * hw + up * hh),
            Vec3.from_array(c - right * hw + up * hh),
        ]

    def signed_distance(self, point: Vec3) -> float:
        """Signed distance from point to gate plane (positive = correct side)."""
        diff = point - self.center
        return diff.dot(self.normal)

    def is_within_bounds(self, point: Vec3, margin: float = 0.0) -> bool:
        """Check if a point projected onto the gate plane is within gate bounds."""
        n = self.normal.to_array()
        n = n / np.linalg.norm(n)

        up_candidate = np.array([0, 0, 1])
        if abs(np.dot(n, up_candidate)) > 0.99:
            up_candidate = np.array([0, 1, 0])

        right = np.cross(n, up_candidate)
        right = right / np.linalg.norm(right)
        up = np.cross(right, n)
        up = up / np.linalg.norm(up)

        diff = (point - self.center).to_array()
        proj_right = abs(np.dot(diff, right))
        proj_up = abs(np.dot(diff, up))

        return (proj_right <= (self.width / 2 + margin) and
                proj_up <= (self.height / 2 + margin))

    def check_passage(self, prev_pos: Vec3, curr_pos: Vec3) -> bool:
        """
        Check if the drone crossed this gate between two positions.
        Returns True if the drone crossed the gate plane in the correct
        direction and within gate bounds.
        """
        d_prev = self.signed_distance(prev_pos)
        d_curr = self.signed_distance(curr_pos)

        # Must cross from negative to positive side (or very close to plane)
        if not (d_prev <= 0 and d_curr > 0):
            return False

        # Interpolate crossing point on the plane
        total = abs(d_prev) + abs(d_curr)
        if total < 1e-10:
            crossing_point = curr_pos
        else:
            t = abs(d_prev) / total
            p = prev_pos.to_array() + t * (curr_pos.to_array() - prev_pos.to_array())
            crossing_point = Vec3.from_array(p)

        # Check if crossing point is within gate bounds (with small margin)
        return self.is_within_bounds(crossing_point, margin=0.3)


@dataclass
class RaceCourse:
    """A complete racecourse with ordered gates."""
    name: str
    gates: List[Gate] = field(default_factory=list)
    start_position: Vec3 = field(default_factory=Vec3)
    start_orientation: Quaternion = field(default_factory=Quaternion)
    time_limit: float = 120.0  # seconds

    @property
    def num_gates(self) -> int:
        return len(self.gates)

    def get_next_gate(self, current_index: int) -> Optional[Gate]:
        if current_index < len(self.gates):
            return self.gates[current_index]
        return None

    def get_lookahead_gates(self, current_index: int, count: int = 3) -> List[Gate]:
        """Get the next N gates for path planning."""
        end = min(current_index + count, len(self.gates))
        return self.gates[current_index:end]

    def reset(self):
        """Reset all gate passage states."""
        for gate in self.gates:
            gate.passed = False
            gate.pass_time = None


# ─── Course Factory ─────────────────────────────────────────────────

def create_oval_course(
    num_gates: int = 10,
    radius: float = 40.0,
    altitude: float = -10.0,  # NED: negative is up
    gate_size: float = 3.0,
    name: str = "Oval Course"
) -> RaceCourse:
    """Create an oval/circular racecourse."""
    gates = []
    for i in range(num_gates):
        angle = (2 * np.pi * i) / num_gates
        next_angle = (2 * np.pi * (i + 1)) / num_gates

        cx = radius * np.cos(angle)
        cy = radius * np.sin(angle)

        # Normal points toward next gate (tangent to circle)
        nx = np.cos(angle + np.pi / 2)
        ny = np.sin(angle + np.pi / 2)

        gates.append(Gate(
            index=i,
            center=Vec3(cx, cy, altitude),
            normal=Vec3(nx, ny, 0).normalized(),
            width=gate_size,
            height=gate_size,
        ))

    # Start just before gate 0, facing it
    start_pos = Vec3(
        radius * np.cos(-0.1),
        radius * np.sin(-0.1),
        altitude
    )

    return RaceCourse(
        name=name,
        gates=gates,
        start_position=start_pos,
        start_orientation=Quaternion.from_euler(0, 0, np.pi / 2),
    )


def create_figure8_course(
    num_gates: int = 16,
    radius: float = 30.0,
    altitude: float = -10.0,
    gate_size: float = 3.0,
    name: str = "Figure-8 Course"
) -> RaceCourse:
    """Create a figure-8 racecourse with elevation changes."""
    gates = []
    half = num_gates // 2

    for i in range(num_gates):
        t = (2 * np.pi * i) / num_gates

        # Lemniscate of Bernoulli (figure 8)
        denom = 1 + np.sin(t) ** 2
        cx = radius * np.cos(t) / denom
        cy = radius * np.sin(t) * np.cos(t) / denom
        cz = altitude + 3.0 * np.sin(2 * t)  # Altitude variation

        # Compute tangent for gate normal
        dt = 0.01
        t2 = t + dt
        denom2 = 1 + np.sin(t2) ** 2
        nx = (radius * np.cos(t2) / denom2 - cx)
        ny = (radius * np.sin(t2) * np.cos(t2) / denom2 - cy)
        nz = (altitude + 3.0 * np.sin(2 * t2) - cz)

        normal = Vec3(nx, ny, nz).normalized()

        gates.append(Gate(
            index=i,
            center=Vec3(cx, cy, cz),
            normal=normal,
            width=gate_size,
            height=gate_size,
        ))

    start_pos = Vec3(gates[0].center.x - 5, gates[0].center.y, gates[0].center.z)

    return RaceCourse(
        name=name,
        gates=gates,
        start_position=start_pos,
        time_limit=180.0,
    )


def create_sprint_course(
    num_gates: int = 8,
    spacing: float = 15.0,
    altitude: float = -8.0,
    gate_size: float = 3.0,
    name: str = "Sprint Course"
) -> RaceCourse:
    """Create a straight sprint course with lateral offsets and height changes."""
    gates = []
    for i in range(num_gates):
        # Add some lateral and vertical variation
        lateral = 5.0 * np.sin(i * 0.8)
        vert_offset = 2.0 * np.sin(i * 1.2)

        cx = i * spacing
        cy = lateral
        cz = altitude + vert_offset

        # Normal points generally forward (+X) with slight adjustment
        if i < num_gates - 1:
            next_lateral = 5.0 * np.sin((i + 1) * 0.8)
            next_vert = 2.0 * np.sin((i + 1) * 1.2)
            nx = spacing
            ny = next_lateral - lateral
            nz = next_vert - vert_offset
            normal = Vec3(nx, ny, nz).normalized()
        else:
            normal = Vec3(1, 0, 0)

        gates.append(Gate(
            index=i,
            center=Vec3(cx, cy, cz),
            normal=normal,
            width=gate_size,
            height=gate_size,
        ))

    start_pos = Vec3(-10, 0, altitude)

    return RaceCourse(
        name=name,
        gates=gates,
        start_position=start_pos,
    )


# Course registry
COURSES = {
    "oval": create_oval_course,
    "figure8": create_figure8_course,
    "sprint": create_sprint_course,
}

def get_course(name: str, **kwargs) -> RaceCourse:
    """Get a course by name with optional overrides."""
    if name not in COURSES:
        raise ValueError(f"Unknown course: {name}. Available: {list(COURSES.keys())}")
    return COURSES[name](**kwargs)