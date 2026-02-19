"""
Path Planning Module — Trajectory generation through gate sequences.

Implements multiple planning strategies:
1. Waypoint-based: Direct gate-to-gate with smoothing
2. Spline-based: Cubic/B-spline through gate centers for smooth trajectories
3. Racing-line: Optimized trajectories that cut corners for speed

The planner outputs a list of trajectory waypoints that the controller follows.
"""

import numpy as np
from scipy.interpolate import CubicSpline, splprep, splev
from typing import List, Tuple, Optional
from dataclasses import dataclass, field
from core.drone_state import Vec3, DroneState
from core.racecourse import Gate, RaceCourse


@dataclass
class TrajectoryPoint:
    """A single point on a planned trajectory."""
    position: Vec3
    velocity_hint: Optional[Vec3] = None  # Desired velocity at this point
    speed_limit: float = 15.0             # Max speed at this point (m/s)
    is_gate_point: bool = False           # True if this is a gate center
    gate_index: Optional[int] = None


@dataclass
class Trajectory:
    """A complete planned trajectory."""
    points: List[TrajectoryPoint] = field(default_factory=list)
    total_distance: float = 0.0

    def get_nearest_point_index(self, position: Vec3) -> int:
        """Find the index of the nearest trajectory point."""
        min_dist = float('inf')
        min_idx = 0
        for i, tp in enumerate(self.points):
            d = position.distance_to(tp.position)
            if d < min_dist:
                min_dist = d
                min_idx = i
        return min_idx

    def get_lookahead_point(self, position: Vec3, lookahead_dist: float = 5.0) -> TrajectoryPoint:
        """
        Get a point on the trajectory ahead of current position.
        Pure pursuit style lookahead.
        """
        nearest_idx = self.get_nearest_point_index(position)

        # Walk forward along trajectory until we exceed lookahead distance
        accumulated = 0.0
        for i in range(nearest_idx, len(self.points) - 1):
            seg_len = self.points[i].position.distance_to(self.points[i + 1].position)
            accumulated += seg_len
            if accumulated >= lookahead_dist:
                return self.points[i + 1]

        # If we run out of points, return the last one
        return self.points[-1]


class PathPlanner:
    """Base path planner interface."""

    def plan(self, state: DroneState, course: RaceCourse, lookahead_gates: int = 3) -> Trajectory:
        raise NotImplementedError


class WaypointPlanner(PathPlanner):
    """
    Simple waypoint planner: generates intermediate points between gates
    with approach vectors aligned to gate normals.
    """

    def __init__(self, approach_distance: float = 5.0, points_per_segment: int = 5):
        self.approach_distance = approach_distance
        self.points_per_segment = points_per_segment

    def plan(self, state: DroneState, course: RaceCourse, lookahead_gates: int = 3) -> Trajectory:
        gates = course.get_lookahead_gates(state.current_gate_index, lookahead_gates)
        if not gates:
            return Trajectory()

        points = []
        current_pos = state.position

        for gate in gates:
            # Create approach point before the gate
            approach_offset = gate.normal * (-self.approach_distance)
            approach_pos = gate.center + approach_offset

            # Interpolate from current position to approach point
            for j in range(1, self.points_per_segment + 1):
                t = j / self.points_per_segment
                interp = Vec3.from_array(
                    current_pos.to_array() * (1 - t) + approach_pos.to_array() * t
                )
                points.append(TrajectoryPoint(
                    position=interp,
                    speed_limit=12.0,
                ))

            # Gate center point
            points.append(TrajectoryPoint(
                position=gate.center,
                speed_limit=8.0,  # Slow down through gates for precision
                is_gate_point=True,
                gate_index=gate.index,
            ))

            # Exit point past the gate
            exit_offset = gate.normal * self.approach_distance
            exit_pos = gate.center + exit_offset
            points.append(TrajectoryPoint(
                position=exit_pos,
                speed_limit=12.0,
            ))

            current_pos = exit_pos

        # Compute total distance
        total = 0.0
        for i in range(1, len(points)):
            total += points[i - 1].position.distance_to(points[i].position)

        return Trajectory(points=points, total_distance=total)


class SplinePlanner(PathPlanner):
    """
    Smooth spline trajectory through gate centers.
    Uses cubic splines for C2 continuity — smooth position, velocity,
    and acceleration profiles.
    """

    def __init__(self, points_per_segment: int = 20, gate_speed: float = 10.0,
                 cruise_speed: float = 15.0):
        self.points_per_segment = points_per_segment
        self.gate_speed = gate_speed
        self.cruise_speed = cruise_speed

    def plan(self, state: DroneState, course: RaceCourse, lookahead_gates: int = 5) -> Trajectory:
        gates = course.get_lookahead_gates(state.current_gate_index, lookahead_gates)
        if len(gates) < 2:
            # Fall back to waypoint planner for single gates
            return WaypointPlanner().plan(state, course, lookahead_gates)

        # Build control points: current position + gate centers
        control_points = [state.position.to_array()]
        gate_indices_in_spline = []

        for gate in gates:
            # Add approach point
            approach = gate.center + gate.normal * (-3.0)
            control_points.append(approach.to_array())

            # Add gate center
            gate_indices_in_spline.append(len(control_points))
            control_points.append(gate.center.to_array())

            # Add exit point
            exit_pt = gate.center + gate.normal * 3.0
            control_points.append(exit_pt.to_array())

        control_points = np.array(control_points)

        # Fit parametric spline
        try:
            n_pts = len(control_points)
            if n_pts < 4:
                # Not enough points for cubic spline, use linear interpolation
                return self._linear_interpolation(control_points, gates)

            # Parametric spline fitting
            tck, u = splprep(
                [control_points[:, 0], control_points[:, 1], control_points[:, 2]],
                s=0.1,  # Tighter fit for accuracy
                k=min(3, n_pts - 1)
            )

            # Sample the spline
            total_points = self.points_per_segment * len(gates)
            u_new = np.linspace(0, 1, total_points)
            x_new, y_new, z_new = splev(u_new, tck)

        except Exception:
            # Fallback to waypoint planner
            return WaypointPlanner().plan(state, course, lookahead_gates)

        # Build trajectory points
        points = []
        for i in range(len(x_new)):
            pos = Vec3(float(x_new[i]), float(y_new[i]), float(z_new[i]))

            # Check if this point is near a gate center
            is_gate = False
            gate_idx = None
            for gate in gates:
                if pos.distance_to(gate.center) < 2.0:
                    is_gate = True
                    gate_idx = gate.index
                    break

            speed = self.gate_speed if is_gate else self.cruise_speed

            # Compute velocity hint from spline tangent
            vel_hint = None
            if i < len(x_new) - 1:
                dx = x_new[i + 1] - x_new[i]
                dy = y_new[i + 1] - y_new[i]
                dz = z_new[i + 1] - z_new[i]
                vel_hint = Vec3(dx, dy, dz).normalized() * speed

            points.append(TrajectoryPoint(
                position=pos,
                velocity_hint=vel_hint,
                speed_limit=speed,
                is_gate_point=is_gate,
                gate_index=gate_idx,
            ))

        total_dist = sum(
            points[i].position.distance_to(points[i + 1].position)
            for i in range(len(points) - 1)
        )

        return Trajectory(points=points, total_distance=total_dist)

    def _linear_interpolation(self, control_points: np.ndarray, gates: List[Gate]) -> Trajectory:
        """Fallback linear interpolation."""
        points = []
        for i in range(len(control_points) - 1):
            for t in np.linspace(0, 1, 10, endpoint=(i == len(control_points) - 2)):
                pos = control_points[i] * (1 - t) + control_points[i + 1] * t
                points.append(TrajectoryPoint(position=Vec3.from_array(pos)))
        return Trajectory(points=points)


class RacingLinePlanner(PathPlanner):
    """
    Optimized racing line planner.
    Cuts corners by shifting gate approach/exit vectors to minimize
    total path curvature while maintaining valid gate passage.
    
    This is the competitive-edge planner for speed optimization.
    """

    def __init__(self, corner_cut_factor: float = 0.7, points_per_segment: int = 25,
                 min_gate_speed: float = 8.0, max_speed: float = 18.0):
        self.corner_cut_factor = corner_cut_factor  # 0=no cut, 1=max cut
        self.points_per_segment = points_per_segment
        self.min_gate_speed = min_gate_speed
        self.max_speed = max_speed

    def plan(self, state: DroneState, course: RaceCourse, lookahead_gates: int = 5) -> Trajectory:
        gates = course.get_lookahead_gates(state.current_gate_index, lookahead_gates)
        if len(gates) < 2:
            return WaypointPlanner().plan(state, course, lookahead_gates)

        # Compute racing line control points
        control_points = [state.position.to_array()]

        for i, gate in enumerate(gates):
            # Compute optimal crossing point within gate bounds
            # Shift toward the inside of the turn
            if i > 0 and i < len(gates) - 1:
                prev_center = gates[i - 1].center.to_array()
                next_center = gates[i + 1].center.to_array()
                curr_center = gate.center.to_array()

                # Direction from prev to next (chord)
                chord = next_center - prev_center
                chord_norm = chord / (np.linalg.norm(chord) + 1e-10)

                # Perpendicular to chord in the gate plane
                gate_normal = gate.normal.to_array()
                gate_right = np.cross(gate_normal, np.array([0, 0, 1]))
                if np.linalg.norm(gate_right) < 0.1:
                    gate_right = np.cross(gate_normal, np.array([0, 1, 0]))
                gate_right = gate_right / (np.linalg.norm(gate_right) + 1e-10)

                # Shift crossing point toward inside of turn
                shift = np.dot(chord_norm, gate_right) * gate_right
                max_shift = gate.width * 0.3  # Don't shift more than 30% of gate width
                shift = np.clip(shift, -max_shift, max_shift) * self.corner_cut_factor

                crossing = curr_center + shift
            else:
                crossing = gate.center.to_array()

            # Short approach and exit segments
            n = gate.normal.to_array()
            approach = crossing - n * 2.0
            exit_pt = crossing + n * 2.0

            control_points.append(approach)
            control_points.append(crossing)
            control_points.append(exit_pt)

        control_points = np.array(control_points)

        # Fit smooth spline through racing line
        try:
            n_pts = len(control_points)
            k = min(3, n_pts - 1)
            tck, u = splprep(
                [control_points[:, 0], control_points[:, 1], control_points[:, 2]],
                s=1.0,
                k=k
            )
            total_points = self.points_per_segment * len(gates)
            u_new = np.linspace(0, 1, total_points)
            x_new, y_new, z_new = splev(u_new, tck)

        except Exception:
            return SplinePlanner().plan(state, course, lookahead_gates)

        # Compute speed profile based on curvature
        points = []
        for i in range(len(x_new)):
            pos = Vec3(float(x_new[i]), float(y_new[i]), float(z_new[i]))

            # Estimate curvature for speed limit
            if 1 <= i < len(x_new) - 1:
                p0 = np.array([x_new[i-1], y_new[i-1], z_new[i-1]])
                p1 = np.array([x_new[i], y_new[i], z_new[i]])
                p2 = np.array([x_new[i+1], y_new[i+1], z_new[i+1]])
                curvature = self._estimate_curvature(p0, p1, p2)
                # Higher curvature = lower speed
                speed = max(self.min_gate_speed,
                           self.max_speed * (1 - curvature * 5.0))
            else:
                speed = self.max_speed

            # Check gate proximity
            is_gate = False
            gate_idx = None
            for gate in gates:
                if pos.distance_to(gate.center) < 2.0:
                    is_gate = True
                    gate_idx = gate.index
                    speed = min(speed, self.min_gate_speed + 4.0)
                    break

            points.append(TrajectoryPoint(
                position=pos,
                speed_limit=speed,
                is_gate_point=is_gate,
                gate_index=gate_idx,
            ))

        total_dist = sum(
            points[i].position.distance_to(points[i + 1].position)
            for i in range(len(points) - 1)
        )

        return Trajectory(points=points, total_distance=total_dist)

    @staticmethod
    def _estimate_curvature(p0: np.ndarray, p1: np.ndarray, p2: np.ndarray) -> float:
        """Estimate curvature at p1 using discrete Menger curvature."""
        d01 = np.linalg.norm(p1 - p0)
        d12 = np.linalg.norm(p2 - p1)
        d02 = np.linalg.norm(p2 - p0)

        if d01 < 1e-10 or d12 < 1e-10 or d02 < 1e-10:
            return 0.0

        # Area of triangle
        cross = np.cross(p1 - p0, p2 - p0)
        area = np.linalg.norm(cross) / 2.0

        # Menger curvature = 4*area / (d01*d12*d02)
        curvature = 4 * area / (d01 * d12 * d02)
        return float(curvature)


# ─── Planner Registry ────────────────────────────────────────────
PLANNERS = {
    "waypoint": WaypointPlanner,
    "spline": SplinePlanner,
    "racing": RacingLinePlanner,
}

def get_planner(name: str = "spline", **kwargs) -> PathPlanner:
    """Get a planner by name."""
    if name not in PLANNERS:
        raise ValueError(f"Unknown planner: {name}. Available: {list(PLANNERS.keys())}")
    return PLANNERS[name](**kwargs)