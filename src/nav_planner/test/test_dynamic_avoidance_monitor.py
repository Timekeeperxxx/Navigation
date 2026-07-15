from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path


SCRIPT_PATH = Path(__file__).parents[1] / "script" / "dynamic_avoidance_monitor.py"
SPEC = importlib.util.spec_from_file_location("dynamic_avoidance_monitor_under_test", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)

Point3 = MODULE.Point3


class MonitorHarness:
    """Run the geometry check without starting a ROS node on the live robot."""

    path_corridor_radius = 0.10
    z_tolerance = 0.10
    robot_self_clear_radius = 0.90
    path_deviation_tolerance = 1.0
    lookahead_distance = 2.0
    slow_distance = 1.8
    slow_nearby_obstacles = False

    _distance_2d = MODULE.DynamicAvoidanceMonitor._distance_2d
    _distance_to_path = MODULE.DynamicAvoidanceMonitor._distance_to_path
    _min_optional = MODULE.DynamicAvoidanceMonitor._min_optional
    _check_obstacles = MODULE.DynamicAvoidanceMonitor._check_obstacles

    def __init__(self, obstacles: list[Point3]) -> None:
        self.obstacles = obstacles


ROBOT = Point3(0.0, 0.0, 0.0)
PATH = [Point3(0.0, 0.0, 0.30), Point3(2.0, 0.0, 0.30)]


def test_floor_voxel_below_lifted_path_does_not_block() -> None:
    monitor = MonitorHarness([Point3(1.0, 0.0, 0.05)])

    result = monitor._check_obstacles(ROBOT, PATH)

    assert result.path_blocked is False
    assert result.blocker_count == 0


def test_preinflated_cloud_gets_only_voxel_margin() -> None:
    monitor = MonitorHarness([Point3(1.0, 0.20, 0.30)])

    result = monitor._check_obstacles(ROBOT, PATH)

    assert result.path_blocked is False
    assert result.nearest_path_distance == 0.20


def test_external_inflated_voxel_on_execution_path_still_blocks() -> None:
    monitor = MonitorHarness([Point3(1.0, 0.05, 0.30)])

    result = monitor._check_obstacles(ROBOT, PATH)

    assert result.path_blocked is True
    assert result.blocker_count == 1
    assert math.isclose(result.nearest_blocker_distance, math.hypot(1.0, 0.05))


def test_robot_self_occupancy_remains_filtered() -> None:
    monitor = MonitorHarness([Point3(0.80, 0.0, 0.30)])

    result = monitor._check_obstacles(ROBOT, PATH)

    assert result.path_blocked is False
    assert result.self_filtered_count == 1


def test_points_beyond_active_path_window_are_pruned_before_segment_checks() -> None:
    monitor = MonitorHarness([Point3(4.0, 0.0, 0.30)])

    result = monitor._check_obstacles(ROBOT, PATH)

    assert result.path_blocked is False
    assert result.nearest_obstacle_distance is None
