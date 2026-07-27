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
PathWindow = MODULE.PathWindow
ObstacleCheck = MODULE.ObstacleCheck


class MonitorHarness:
    """Run the geometry check without starting a ROS node on the live robot."""

    path_corridor_radius = 0.10
    z_tolerance = 0.10
    z_tolerance_up = 0.45
    robot_self_clear_radius = 0.90
    path_deviation_tolerance = 1.0
    lookahead_distance = 2.0
    slow_distance = 1.8
    slow_nearby_obstacles = False
    allow_clearance_escape = True
    clearance_escape_max_distance = 0.60
    clearance_escape_max_drop = 0.015
    clearance_escape_min_improvement = 0.02
    footprint_center_offsets = (-0.22, -0.63)
    footprint_sweep_max_step = 0.05
    footprint_sweep_max_yaw_step = math.radians(5.0)
    robot_yaw = 0.0

    _distance_2d = MODULE.DynamicAvoidanceMonitor._distance_2d
    _distance_3d = MODULE.DynamicAvoidanceMonitor._distance_3d
    _distance_to_path = MODULE.DynamicAvoidanceMonitor._distance_to_path
    _distances_to_path = MODULE.DynamicAvoidanceMonitor._distances_to_path
    _is_clearance_escape = MODULE.DynamicAvoidanceMonitor._is_clearance_escape
    _min_optional = MODULE.DynamicAvoidanceMonitor._min_optional
    _normalize_angle = staticmethod(
        MODULE.DynamicAvoidanceMonitor._normalize_angle
    )
    _path_with_robot_start = MODULE.DynamicAvoidanceMonitor._path_with_robot_start
    _estimate_path_yaws = MODULE.DynamicAvoidanceMonitor._estimate_path_yaws
    _footprint_sweep_paths = MODULE.DynamicAvoidanceMonitor._footprint_sweep_paths
    _check_obstacles = MODULE.DynamicAvoidanceMonitor._check_obstacles

    def __init__(self, obstacles: list[Point3]) -> None:
        self.obstacles = obstacles


class DecisionHarness:
    enforce_path_blocking = True
    replan_blocked_duration = 2.0
    blocked_clear_duration = 2.0
    transient_clear_duration = 0.4
    stop_distance = 1.2
    slow_distance = 1.8
    slow_nearby_obstacles = False
    active_path_source = "scan_execution_path"
    execution_path_generation_time = 10.0

    _decide_status = MODULE.DynamicAvoidanceMonitor._decide_status
    _remember_blocked_execution_generation = (
        MODULE.DynamicAvoidanceMonitor._remember_blocked_execution_generation
    )
    _has_new_execution_generation = (
        MODULE.DynamicAvoidanceMonitor._has_new_execution_generation
    )
    _reset_block_tracking = MODULE.DynamicAvoidanceMonitor._reset_block_tracking

    def __init__(self) -> None:
        self.blocked_since = None
        self.clear_since = None
        self.blocked_execution_generation_time = None
        self.replan_confirmation_required = False
        self.replan_active = False
        self.replan_events: list[bool] = []

    def _publish_replan(self, active: bool, _now: float) -> None:
        self.replan_active = active
        self.replan_events.append(active)

    def _status_payload(
        self,
        status: str,
        action: str,
        path_blocked: bool,
        _now: float,
        **_kwargs: object,
    ) -> dict[str, object]:
        return {
            "status": status,
            "action": action,
            "path_blocked": path_blocked,
        }


class StreamHarness:
    _obstacle_stream_lost = (
        MODULE.DynamicAvoidanceMonitor._obstacle_stream_lost
    )

    def __init__(
        self,
        *,
        heartbeat_topic: str,
        heartbeat_time: float | None,
        obstacle_time: float | None,
    ) -> None:
        self.require_obstacle_stream = True
        self.sensor_heartbeat_topic = heartbeat_topic
        self.last_sensor_heartbeat_time = heartbeat_time
        self.last_obstacle_time = obstacle_time
        self.sensor_timeout = 1.5


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


def test_overhang_at_lidar_height_blocks() -> None:
    # A table edge or shelf beam: physical bottom near 1.0 m appears in the
    # inflated cloud from about 0.70 m.  The old symmetric ±0.10 band ignored
    # it and the B2 walked its Mid360 mast straight into it.
    monitor = MonitorHarness([Point3(1.0, 0.0, 0.70)])

    result = monitor._check_obstacles(ROBOT, PATH)

    assert result.path_blocked is True
    assert result.blocker_count == 1


def test_clearance_above_robot_top_does_not_block() -> None:
    # Inflated voxel at 0.80 m above ground maps to a physical overhang above
    # about 1.13 m, which the 0.97 m machine passes underneath.
    monitor = MonitorHarness([Point3(1.0, 0.0, 0.80)])

    result = monitor._check_obstacles(ROBOT, PATH)

    assert result.path_blocked is False
    assert result.blocker_count == 0


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


def test_inner_corner_rear_circle_sweep_blocks_before_body_origin_path() -> None:
    corner_path = [
        Point3(0.0, 0.0, 0.30),
        Point3(1.0, 0.0, 0.30),
        Point3(1.0, 1.0, 0.30),
    ]
    # At the 90-degree corner the rear circle swings through this point while
    # the body-origin polyline remains more than the corridor margin away.
    obstacle = Point3(0.55, -0.45, 0.30)
    monitor = MonitorHarness([obstacle])
    monitor.robot_self_clear_radius = 0.10

    origin_distance, _ = monitor._distance_to_path(obstacle, corner_path)
    result = monitor._check_obstacles(ROBOT, corner_path)

    assert origin_distance > monitor.path_corridor_radius
    assert result.path_blocked is True
    assert result.nearest_path_distance is not None
    assert result.nearest_path_distance < 0.03


def test_live_robot_yaw_is_included_in_initial_footprint_sweep() -> None:
    monitor = MonitorHarness([Point3(0.0, 0.63, 0.30)])
    monitor.robot_self_clear_radius = 0.10
    monitor.robot_yaw = -math.pi / 2.0

    result = monitor._check_obstacles(ROBOT, PATH)

    assert result.path_blocked is True


def test_clearance_escape_only_accepts_path_that_moves_out_from_current_boundary() -> None:
    monitor = MonitorHarness([Point3(-0.63, 0.0, 0.30)])
    monitor.robot_self_clear_radius = 0.10

    result = monitor._check_obstacles(ROBOT, PATH)

    assert result.path_blocked is True
    assert result.clearance_escape is True


def test_clearance_escape_does_not_bypass_future_obstacle() -> None:
    monitor = MonitorHarness([Point3(0.50, 0.0, 0.30)])
    monitor.robot_self_clear_radius = 0.10

    result = monitor._check_obstacles(ROBOT, PATH)

    assert result.path_blocked is True
    assert result.clearance_escape is False


def test_persistent_block_waits_for_new_continuously_clear_trajectory() -> None:
    monitor = DecisionHarness()
    window = PathWindow(PATH, PATH, 0, 0.0)
    blocked = ObstacleCheck(True, 1, 0, 0.5, 0.5, 0.0)
    clear = ObstacleCheck(False, 0, 0, 0.5, None, 0.2)

    assert monitor._decide_status(0.0, ROBOT, window, blocked)["status"] == "blocked"
    assert (
        monitor._decide_status(2.1, ROBOT, window, blocked)["status"]
        == "replan_requested"
    )
    assert (
        monitor._decide_status(2.2, ROBOT, window, clear)["status"]
        == "waiting_replan"
    )

    monitor.execution_path_generation_time = 11.0
    assert monitor._decide_status(2.3, ROBOT, window, clear)["status"] == "clearing"
    assert monitor.replan_active is False
    assert monitor.replan_events == [True, False]
    assert monitor._decide_status(3.0, ROBOT, window, clear)["status"] == "clearing"
    assert monitor._decide_status(4.4, ROBOT, window, clear)["status"] == "clear"
    assert monitor.replan_events == [True, False]


def test_transient_block_uses_short_clear_confirmation_without_replan() -> None:
    monitor = DecisionHarness()
    window = PathWindow(PATH, PATH, 0, 0.0)
    blocked = ObstacleCheck(True, 1, 0, 0.5, 0.5, 0.0)
    clear = ObstacleCheck(False, 0, 0, 0.5, None, 0.2)

    assert monitor._decide_status(0.0, ROBOT, window, blocked)["status"] == "blocked"
    assert monitor._decide_status(0.1, ROBOT, window, clear)["status"] == "clearing"
    assert monitor._decide_status(0.3, ROBOT, window, clear)["status"] == "clearing"
    assert monitor._decide_status(0.6, ROBOT, window, clear)["status"] == "clear"
    assert monitor.replan_events == []


def test_raw_sensor_heartbeat_timeout_is_not_hidden_by_republished_grid() -> None:
    monitor = StreamHarness(
        heartbeat_topic="/lio/cloud_world",
        heartbeat_time=8.0,
        obstacle_time=9.9,
    )

    assert monitor._obstacle_stream_lost(10.0) is True


def test_recent_raw_sensor_heartbeat_keeps_stream_healthy() -> None:
    monitor = StreamHarness(
        heartbeat_topic="/lio/cloud_world",
        heartbeat_time=9.0,
        obstacle_time=8.0,
    )

    assert monitor._obstacle_stream_lost(10.0) is False
