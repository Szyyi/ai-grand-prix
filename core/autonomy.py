"""
Autonomy Stack — The competition entry point.

This is the brain. It connects perception → planning → control
in a single callable that the simulator or competition adapter invokes
each tick.

When the real competition interface is released, only the adapter layer
(competition_adapter.py) needs to change. This core logic stays the same.
"""

import numpy as np
from typing import Optional
from core.drone_state import DroneState, Vec3
from core.racecourse import RaceCourse, Gate
from planning.trajectory import (
    PathPlanner, SplinePlanner, RacingLinePlanner, WaypointPlanner,
    Trajectory, TrajectoryPoint
)
from control.flight_controller import FlightController, FlightControllerConfig, ControlCommand


class AutonomyConfig:
    """Top-level autonomy tuning parameters."""

    def __init__(self):
        # Planning
        self.planner_type: str = "racing"    # "waypoint", "spline", "racing"
        self.lookahead_gates: int = 4
        self.replan_interval: float = 0.3    # Replan every N seconds
        self.replan_on_gate_pass: bool = True

        # Control
        self.controller_config = FlightControllerConfig()

        # Pure pursuit
        self.lookahead_distance: float = 6.0  # meters ahead on trajectory
        self.min_lookahead: float = 3.0
        self.max_lookahead: float = 12.0
        self.speed_lookahead_gain: float = 0.4  # lookahead scales with speed

        # Gate approach
        self.gate_approach_speed: float = 10.0    # Slow down near gates
        self.gate_slow_distance: float = 6.0      # Start slowing at this distance
        self.cruise_speed: float = 18.0

        # Safety
        self.min_altitude: float = 2.0     # meters AGL
        self.max_altitude: float = 50.0
        self.recovery_mode_threshold: float = 30.0  # degrees tilt triggers recovery


class AutonomyStack:
    """
    Main autonomy stack.
    Call .update(state, course) each tick to get a control command.
    """

    def __init__(self, config: Optional[AutonomyConfig] = None):
        self.config = config or AutonomyConfig()

        # Initialize planner
        if self.config.planner_type == "racing":
            self.planner = RacingLinePlanner()
        elif self.config.planner_type == "spline":
            self.planner = SplinePlanner()
        else:
            self.planner = WaypointPlanner()

        # Initialize controller
        self.controller = FlightController(self.config.controller_config)

        # State
        self.trajectory: Optional[Trajectory] = None
        self.last_replan_time: float = 0.0
        self.last_gate_index: int = 0
        self.mode: str = "racing"  # "racing", "recovery", "finished"
        self.tick_count: int = 0

    def update(self, state: DroneState, course: RaceCourse) -> ControlCommand:
        """
        Main autonomy loop — called every simulation tick.

        Args:
            state: Current observed drone state
            course: The racecourse

        Returns:
            ControlCommand for the flight controller
        """
        self.tick_count += 1

        # ── Check race completion ──
        if state.race_finished or state.current_gate_index >= course.num_gates:
            self.mode = "finished"
            return self._hover_command(state)

        # ── Safety checks ──
        if self._needs_recovery(state):
            self.mode = "recovery"
            return self._recovery_command(state)
        else:
            self.mode = "racing"

        # ── Replan trajectory if needed ──
        needs_replan = (
            self.trajectory is None or
            len(self.trajectory.points) == 0 or
            (state.race_time - self.last_replan_time) > self.config.replan_interval or
            (self.config.replan_on_gate_pass and state.current_gate_index != self.last_gate_index)
        )

        if needs_replan:
            self.trajectory = self.planner.plan(
                state, course, self.config.lookahead_gates
            )
            self.last_replan_time = state.race_time
            self.last_gate_index = state.current_gate_index

        # ── Stall detection: if we're barely moving, force replan with simpler planner ──
        if (state.speed() < 1.0 and state.race_time > 3.0 and
                not state.race_finished and self.tick_count % 100 == 0):
            # Fall back to waypoint planner for reliability
            fallback = WaypointPlanner()
            self.trajectory = fallback.plan(state, course, self.config.lookahead_gates)
            self.last_replan_time = state.race_time

        if self.trajectory is None or len(self.trajectory.points) == 0:
            # Emergency: no trajectory, fly toward next gate directly
            next_gate = course.get_next_gate(state.current_gate_index)
            if next_gate:
                return self.controller.compute(
                    state, next_gate.center, speed_limit=self.config.gate_approach_speed
                )
            return self._hover_command(state)

        # ── Compute lookahead point (pure pursuit) ──
        speed = state.speed()
        lookahead = self.config.min_lookahead + speed * self.config.speed_lookahead_gain
        lookahead = np.clip(lookahead, self.config.min_lookahead, self.config.max_lookahead)

        target_point = self.trajectory.get_lookahead_point(state.position, lookahead)

        # ── Compute speed limit ──
        speed_limit = self._compute_speed_limit(state, course, target_point)

        # ── Compute yaw target (face direction of travel) ──
        dx = target_point.position.x - state.position.x
        dy = target_point.position.y - state.position.y
        target_yaw = np.arctan2(dy, dx) if (abs(dx) > 0.5 or abs(dy) > 0.5) else None

        # ── Run controller ──
        dt = 0.01  # Fixed tick rate
        command = self.controller.compute(
            state, target_point.position,
            target_yaw=target_yaw,
            speed_limit=speed_limit,
            dt=dt,
        )

        return command

    def _compute_speed_limit(self, state: DroneState, course: RaceCourse,
                             target: TrajectoryPoint) -> float:
        """Dynamic speed limit based on gate proximity and trajectory."""
        speed_limit = self.config.cruise_speed

        # Use trajectory point speed limit if available
        if target.speed_limit < speed_limit:
            speed_limit = target.speed_limit

        # Slow down near next gate
        next_gate = course.get_next_gate(state.current_gate_index)
        if next_gate:
            dist_to_gate = state.position.distance_to(next_gate.center)
            if dist_to_gate < self.config.gate_slow_distance:
                # Linear speed ramp: full speed at gate_slow_distance, approach_speed at gate
                t = dist_to_gate / self.config.gate_slow_distance
                gate_speed = (
                    self.config.gate_approach_speed +
                    t * (self.config.cruise_speed - self.config.gate_approach_speed)
                )
                speed_limit = min(speed_limit, gate_speed)

        return speed_limit

    def _needs_recovery(self, state: DroneState) -> bool:
        """Check if drone needs recovery mode."""
        roll, pitch, _ = state.orientation.to_euler()
        tilt = max(abs(np.degrees(roll)), abs(np.degrees(pitch)))
        return tilt > self.config.recovery_mode_threshold

    def _recovery_command(self, state: DroneState) -> ControlCommand:
        """Emergency: level the drone and hold position."""
        # Target: current XY, safe altitude, level attitude
        safe_pos = Vec3(
            state.position.x,
            state.position.y,
            min(state.position.z, -self.config.min_altitude)
        )
        return self.controller.compute(state, safe_pos, speed_limit=3.0)

    def _hover_command(self, state: DroneState) -> ControlCommand:
        """Hold current position."""
        return self.controller.compute(
            state, state.position, speed_limit=0.0
        )


def create_autonomy_fn(config: Optional[AutonomyConfig] = None):
    """
    Factory: creates the callable that the simulator expects.

    Usage:
        autonomy = create_autonomy_fn()
        result = run_simulation(course, autonomy)
    """
    stack = AutonomyStack(config)

    def autonomy_fn(state: DroneState, course: RaceCourse) -> ControlCommand:
        return stack.update(state, course)

    return autonomy_fn