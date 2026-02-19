"""
Stress Test Suite — Automated reliability and edge-case testing.

Tests the autonomy stack under various conditions:
1. All course types with all planner types
2. Wind disturbances (light, moderate, heavy)
3. Sensor noise levels
4. Gate size variations (smaller = harder)
5. Motor degradation
6. Speed challenges
7. Consistency (repeated runs)
"""

import sys
import os
import numpy as np
import time
from typing import List, Dict, Tuple

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.drone_state import DroneState
from core.racecourse import (
    RaceCourse, get_course, create_oval_course,
    create_figure8_course, create_sprint_course
)
from core.autonomy import AutonomyConfig, create_autonomy_fn
from simulation.simulator import SimConfig, SimResult, run_simulation


class StressTestResult:
    """Result of a single stress test."""
    def __init__(self, name: str, result: SimResult, elapsed_real: float):
        self.name = name
        self.result = result
        self.elapsed_real = elapsed_real

    def __repr__(self):
        status = "PASS" if self.result.success else "FAIL"
        return (f"  [{status}] {self.name:40s} | "
                f"Gates: {self.result.gates_passed:2d}/{self.result.total_gates:2d} | "
                f"Time: {self.result.race_time:6.1f}s | "
                f"Real: {self.elapsed_real:.1f}s")


def run_test(name: str, course: RaceCourse, sim_config: SimConfig,
             auto_config: AutonomyConfig = None, verbose: bool = False) -> StressTestResult:
    """Run a single test case."""
    auto_fn = create_autonomy_fn(auto_config)
    t0 = time.time()
    result = run_simulation(course, auto_fn, sim_config, verbose=verbose)
    elapsed = time.time() - t0
    return StressTestResult(name, result, elapsed)


def test_all_courses(verbose: bool = False) -> List[StressTestResult]:
    """Test 1: All course types with default settings."""
    print("\n═══ TEST 1: All Course Types ═══")
    results = []
    sim_cfg = SimConfig(max_sim_time=120.0)

    for course_name in ["oval", "sprint", "figure8"]:
        course = get_course(course_name)
        for planner in ["waypoint", "spline", "racing"]:
            auto_cfg = AutonomyConfig()
            auto_cfg.planner_type = planner
            name = f"{course_name}/{planner}"
            r = run_test(name, course, sim_cfg, auto_cfg, verbose)
            print(r)
            results.append(r)

    return results


def test_wind_conditions(verbose: bool = False) -> List[StressTestResult]:
    """Test 2: Wind disturbances."""
    print("\n═══ TEST 2: Wind Conditions ═══")
    results = []
    course = create_oval_course(num_gates=8)

    from core.drone_state import Vec3
    wind_configs = [
        ("no_wind", SimConfig(wind_enabled=False)),
        ("light_wind_2ms", SimConfig(wind_enabled=True, wind_mean=Vec3(2, 0, 0), wind_gust_magnitude=1.0)),
        ("moderate_wind_5ms", SimConfig(wind_enabled=True, wind_mean=Vec3(5, 2, 0), wind_gust_magnitude=3.0)),
        ("heavy_wind_8ms", SimConfig(wind_enabled=True, wind_mean=Vec3(8, 4, 1), wind_gust_magnitude=5.0, wind_gust_frequency=1.0)),
    ]

    for name, sim_cfg in wind_configs:
        sim_cfg.max_sim_time = 120.0
        auto_cfg = AutonomyConfig()
        auto_cfg.planner_type = "racing"
        r = run_test(name, course, sim_cfg, auto_cfg, verbose)
        print(r)
        results.append(r)

    return results


def test_sensor_noise(verbose: bool = False) -> List[StressTestResult]:
    """Test 3: Sensor noise levels."""
    print("\n═══ TEST 3: Sensor Noise ═══")
    results = []
    course = create_oval_course(num_gates=8)

    noise_levels = [
        ("clean_sensors", 0.0, 0.0),
        ("light_noise", 0.05, 0.02),
        ("moderate_noise", 0.15, 0.08),
        ("heavy_noise", 0.30, 0.15),
    ]

    for name, pos_noise, vel_noise in noise_levels:
        sim_cfg = SimConfig(
            max_sim_time=120.0,
            position_noise_std=pos_noise,
            velocity_noise_std=vel_noise,
        )
        r = run_test(name, course, sim_cfg, verbose=verbose)
        print(r)
        results.append(r)

    return results


def test_gate_sizes(verbose: bool = False) -> List[StressTestResult]:
    """Test 4: Varying gate sizes (difficulty)."""
    print("\n═══ TEST 4: Gate Size Difficulty ═══")
    results = []
    sim_cfg = SimConfig(max_sim_time=120.0)

    sizes = [
        ("large_5m", 5.0),
        ("standard_3m", 3.0),
        ("small_2m", 2.0),
        ("tiny_1.5m", 1.5),
    ]

    for name, size in sizes:
        course = create_oval_course(num_gates=8, gate_size=size)
        auto_cfg = AutonomyConfig()
        auto_cfg.planner_type = "racing"
        # Slow down for smaller gates
        if size < 2.5:
            auto_cfg.gate_approach_speed = 5.0
            auto_cfg.cruise_speed = 10.0
        r = run_test(name, course, sim_cfg, auto_cfg, verbose)
        print(r)
        results.append(r)

    return results


def test_consistency(num_runs: int = 5, verbose: bool = False) -> List[StressTestResult]:
    """Test 5: Consistency across repeated runs."""
    print(f"\n═══ TEST 5: Consistency ({num_runs} runs) ═══")
    results = []
    course = create_oval_course(num_gates=10)
    sim_cfg = SimConfig(max_sim_time=120.0)

    times = []
    for i in range(num_runs):
        auto_cfg = AutonomyConfig()
        auto_cfg.planner_type = "racing"
        name = f"consistency_run_{i+1}"
        r = run_test(name, course, sim_cfg, auto_cfg, verbose)
        print(r)
        results.append(r)
        if r.result.success:
            times.append(r.result.race_time)

    if times:
        print(f"\n  Consistency stats: mean={np.mean(times):.2f}s, "
              f"std={np.std(times):.2f}s, "
              f"min={min(times):.2f}s, max={max(times):.2f}s")

    return results


def test_speed_challenge(verbose: bool = False) -> List[StressTestResult]:
    """Test 6: Push for maximum speed."""
    print("\n═══ TEST 6: Speed Optimization ═══")
    results = []
    course = create_sprint_course(num_gates=8, spacing=20.0, gate_size=3.5)
    sim_cfg = SimConfig(max_sim_time=60.0)

    speed_configs = [
        ("conservative", 8.0, 12.0),
        ("moderate", 10.0, 15.0),
        ("aggressive", 12.0, 18.0),
        ("maximum", 14.0, 20.0),
    ]

    for name, gate_speed, cruise_speed in speed_configs:
        auto_cfg = AutonomyConfig()
        auto_cfg.planner_type = "racing"
        auto_cfg.gate_approach_speed = gate_speed
        auto_cfg.cruise_speed = cruise_speed
        r = run_test(name, course, sim_cfg, auto_cfg, verbose)
        print(r)
        results.append(r)

    return results


def run_all(verbose: bool = False):
    """Run the complete stress test suite."""
    print("╔══════════════════════════════════════════════════╗")
    print("║       AI GRAND PRIX — STRESS TEST SUITE         ║")
    print("╚══════════════════════════════════════════════════╝")

    all_results = []
    t0 = time.time()

    all_results.extend(test_all_courses(verbose))
    all_results.extend(test_wind_conditions(verbose))
    all_results.extend(test_sensor_noise(verbose))
    all_results.extend(test_gate_sizes(verbose))
    all_results.extend(test_consistency(verbose=verbose))
    all_results.extend(test_speed_challenge(verbose))

    elapsed = time.time() - t0

    # ── Summary ──
    passed = sum(1 for r in all_results if r.result.success)
    failed = len(all_results) - passed

    print("\n" + "═" * 60)
    print(f"RESULTS: {passed}/{len(all_results)} passed, {failed} failed")
    print(f"Total time: {elapsed:.1f}s")

    if failed > 0:
        print("\nFailed tests:")
        for r in all_results:
            if not r.result.success:
                reason = ""
                if r.result.crashed:
                    reason = f" (crash: {r.result.crash_reason})"
                elif r.result.timed_out:
                    reason = " (timeout)"
                print(f"  ✗ {r.name}{reason}")
    print("═" * 60)

    return all_results


if __name__ == "__main__":
    verbose = "--verbose" in sys.argv or "-v" in sys.argv
    run_all(verbose=verbose)