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


def test_velocity_callbacks_are_isolated_from_cpu_heavy_obstacle_checks() -> None:
    source = SCRIPT_PATH.read_text(encoding="utf-8")

    assert "MultiThreadedExecutor(num_threads=3)" in source
    assert "self.command_callback_group = MutuallyExclusiveCallbackGroup()" in source
    assert "self.path_callback_group = MutuallyExclusiveCallbackGroup()" in source
    assert source.count("callback_group=self.command_callback_group") == 3
    assert source.count("callback_group=self.path_callback_group") == 2


def test_stale_execution_path_uses_lifted_global_safety_path() -> None:
    class ActivePathHarness:
        _active_path = MODULE.DynamicAvoidanceMonitor._active_path
        _execution_path_matches_global = (
            MODULE.DynamicAvoidanceMonitor._execution_path_matches_global
        )
        _distance_3d = MODULE.DynamicAvoidanceMonitor._distance_3d

        execution_path_points = [Point3(0.0, 0.0, 0.30)]
        last_execution_path_time = 1.0
        scan_execution_path_timeout = 0.5
        last_global_path_time = 2.0
        execution_path_generation_time = 1.0
        scan_execution_path_global_tolerance = 1.0
        path_points = [Point3(0.0, 0.0, 0.0), Point3(2.0, 0.0, 0.0)]
        global_safety_path_points = [
            Point3(0.0, 0.0, 0.30),
            Point3(2.0, 0.0, 0.30),
        ]

    monitor = ActivePathHarness()

    path, source = monitor._active_path(2.0)

    assert source == "global_path_lifted"
    assert path == monitor.global_safety_path_points
    assert all(point.z == 0.30 for point in path)


class MonitorHarness:
    """Run the geometry check without starting a ROS node on the live robot."""

    path_corridor_radius = 0.10
    z_tolerance = 0.10
    z_tolerance_down = 0.10
    z_tolerance_up = 0.10
    robot_self_clear_radius = 0.90
    path_deviation_tolerance = 1.0
    lookahead_distance = 2.0
    slow_distance = 1.8
    slow_nearby_obstacles = False
    last_robot_yaw = 0.0

    _distance_2d = MODULE.DynamicAvoidanceMonitor._distance_2d
    _distance_to_path = MODULE.DynamicAvoidanceMonitor._distance_to_path
    _min_optional = MODULE.DynamicAvoidanceMonitor._min_optional
    _body_side = MODULE.DynamicAvoidanceMonitor._body_side
    _within_z_tolerance = MODULE.DynamicAvoidanceMonitor._within_z_tolerance
    _check_obstacles = MODULE.DynamicAvoidanceMonitor._check_obstacles

    def __init__(self, obstacles: list[Point3]) -> None:
        self.obstacles = obstacles


ROBOT = Point3(0.0, 0.0, 0.0)
PATH = [Point3(0.0, 0.0, 0.30), Point3(2.0, 0.0, 0.30)]


def obstacle_check(**overrides: object):
    values = {
        "path_blocked": False,
        "blocker_count": 0,
        "self_filtered_count": 0,
        "nearest_obstacle_distance": None,
        "nearest_obstacle_point": None,
        "nearest_obstacle_dx": None,
        "nearest_obstacle_dy": None,
        "nearest_obstacle_dz": None,
        "nearest_obstacle_body_x": None,
        "nearest_obstacle_body_y": None,
        "nearest_obstacle_side": "none",
        "nearest_blocker_distance": None,
        "nearest_path_distance": None,
        "nearest_path_z_distance": None,
        "nearest_path_z_delta": None,
        "nearest_path_obstacle": None,
        "nearest_path_projection": None,
        "nearest_blocker_obstacle": None,
        "nearest_blocker_path_distance": None,
        "nearest_blocker_z_distance": None,
        "nearest_blocker_z_delta": None,
    }
    values.update(overrides)
    return MODULE.ObstacleCheck(**values)


def test_voxel_downsample_keeps_one_point_per_leaf() -> None:
    raw_points = [
        (0.01, 0.01, 0.01),
        (0.08, 0.09, 0.01),
        (0.11, 0.01, 0.01),
        (-0.01, 0.01, 0.01),
    ]

    downsampled = MODULE.DynamicAvoidanceMonitor._voxel_downsample_points(
        raw_points, 0.10
    )

    assert downsampled == [
        (0.01, 0.01, 0.01),
        (0.11, 0.01, 0.01),
        (-0.01, 0.01, 0.01),
    ]


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


def test_nearest_obstacle_is_diagnostic_not_a_front_stop_sector() -> None:
    monitor = MonitorHarness(
        [
            Point3(1.20, 0.0, 0.30),
            Point3(0.10, 1.10, 0.30),
            Point3(-1.00, 0.0, 0.30),
        ]
    )

    result = monitor._check_obstacles(ROBOT, PATH)

    assert math.isclose(result.nearest_obstacle_distance, 1.00)
    assert result.nearest_obstacle_side == "back"


def test_deviated_scan_execution_path_stops_until_path_recovers() -> None:
    class DeviatedHarness:
        _on_timer = MODULE.DynamicAvoidanceMonitor._on_timer
        _status_payload = MODULE.DynamicAvoidanceMonitor._status_payload
        _drop_stale_obstacles_if_optional = (
            MODULE.DynamicAvoidanceMonitor._drop_stale_obstacles_if_optional
        )

        navigation_enabled = True
        enabled = True
        require_obstacle_stream = False
        last_obstacle_time = None
        global_frame = "map"
        robot_frame = "base_footprint"
        path_points = [Point3(0.0, 0.0, 0.0)]
        execution_path_points = [Point3(0.0, 0.0, 0.0)]
        execution_path_age = 0.1
        execution_path_generation_matches = True
        execution_path_spatially_matches = False
        execution_path_start_distance = None
        execution_path_end_distance = None
        execution_path_length = None
        global_path_span = None
        obstacles: list[Point3] = []

        def __init__(self) -> None:
            self.blocked_since = 1.0
            self.set_payload: dict[str, object] | None = None
            self.status_payload: dict[str, object] | None = None
            self.publish_stop_called = False

        def _now_sec(self) -> float:
            return 10.0

        def _active_path(self, now: float) -> tuple[list[Point3], str]:
            return [Point3(0.0, 0.0, 0.0), Point3(1.0, 0.0, 0.0)], "scan_execution_path"

        def _get_robot_point(self) -> tuple[Point3, str]:
            return Point3(5.0, 5.0, 0.0), ""

        def _make_path_window(
            self, robot: Point3, path_points: list[Point3]
        ) -> None:
            return None

        def _set_state(self, payload: dict[str, object]) -> None:
            self.set_payload = payload

        def _publish_status(self, payload: dict[str, object]) -> None:
            self.status_payload = payload

        def _publish_stop_if_needed(self) -> None:
            self.publish_stop_called = True

    monitor = DeviatedHarness()

    monitor._on_timer()

    assert monitor.blocked_since is None
    assert monitor.set_payload is not None
    assert monitor.set_payload["status"] == "deviated"
    assert monitor.set_payload["action"] == "stop"
    assert monitor.publish_stop_called is True
    assert monitor.status_payload == monitor.set_payload


def test_persistent_blocker_outside_stop_distance_requests_slow_replan() -> None:
    class StatusHarness:
        _decide_status = MODULE.DynamicAvoidanceMonitor._decide_status
        _status_payload = MODULE.DynamicAvoidanceMonitor._status_payload

        global_frame = "map"
        robot_frame = "base_footprint"
        active_path_points = PATH
        active_path_source = "scan_execution_path"
        path_points = PATH
        execution_path_points = PATH
        execution_path_age = 0.1
        execution_path_generation_matches = True
        execution_path_spatially_matches = True
        execution_path_start_distance = 0.0
        execution_path_end_distance = 0.0
        execution_path_length = 2.0
        global_path_span = 2.0
        obstacles = [Point3(0.9, 0.0, 0.30)]
        enforce_path_blocking = True
        stop_on_path_blocked = False
        replan_blocked_duration = 0.3
        stop_distance = 0.5
        slow_nearby_obstacles = False
        _point_payload = MODULE.DynamicAvoidanceMonitor._point_payload

        def __init__(self) -> None:
            self.blocked_since = 1.0
            self.replan_requests: list[bool] = []

        def _publish_replan(self, active: bool, now: float) -> None:
            self.replan_requests.append(active)

    monitor = StatusHarness()
    window = MODULE.PathWindow(PATH, PATH, 0, 0.0)
    check = obstacle_check(
        path_blocked=True,
        blocker_count=1,
        self_filtered_count=0,
        nearest_obstacle_distance=0.9,
        nearest_blocker_distance=0.9,
        nearest_path_distance=0.0,
    )

    payload = monitor._decide_status(1.5, ROBOT, window, check)

    assert payload["status"] == "replan_requested"
    assert payload["action"] == "slow"
    assert payload["path_blocked"] is True
    assert monitor.replan_requests == [True]


def test_close_blocker_still_hard_stops() -> None:
    class StatusHarness:
        _decide_status = MODULE.DynamicAvoidanceMonitor._decide_status
        _status_payload = MODULE.DynamicAvoidanceMonitor._status_payload

        global_frame = "map"
        robot_frame = "base_footprint"
        active_path_points = PATH
        active_path_source = "scan_execution_path"
        path_points = PATH
        execution_path_points = PATH
        execution_path_age = 0.1
        execution_path_generation_matches = True
        execution_path_spatially_matches = True
        execution_path_start_distance = 0.0
        execution_path_end_distance = 0.0
        execution_path_length = 2.0
        global_path_span = 2.0
        obstacles = [Point3(0.4, 0.0, 0.30)]
        enforce_path_blocking = True
        stop_on_path_blocked = False
        replan_blocked_duration = 0.3
        stop_distance = 0.5
        slow_nearby_obstacles = False
        _point_payload = MODULE.DynamicAvoidanceMonitor._point_payload

        def __init__(self) -> None:
            self.blocked_since = None
            self.replan_requests: list[bool] = []

        def _publish_replan(self, active: bool, now: float) -> None:
            self.replan_requests.append(active)

    monitor = StatusHarness()
    window = MODULE.PathWindow(PATH, PATH, 0, 0.0)
    check = obstacle_check(
        path_blocked=True,
        blocker_count=1,
        self_filtered_count=0,
        nearest_obstacle_distance=0.4,
        nearest_blocker_distance=0.4,
        nearest_path_distance=0.0,
    )

    payload = monitor._decide_status(1.5, ROBOT, window, check)

    assert payload["status"] == "blocked"
    assert payload["action"] == "stop"
    assert monitor.replan_requests == [True]


def test_slow_state_publishes_speed_scale_without_freeze() -> None:
    class Publisher:
        def __init__(self) -> None:
            self.messages: list[object] = []

        def publish(self, msg: object) -> None:
            self.messages.append(msg)

    class StateHarness:
        _set_state = MODULE.DynamicAvoidanceMonitor._set_state
        _current_speed_scale = MODULE.DynamicAvoidanceMonitor._current_speed_scale
        _publish_safety_speed_scale = (
            MODULE.DynamicAvoidanceMonitor._publish_safety_speed_scale
        )

        navigation_enabled = True
        slow_speed_scale = 0.9

        def __init__(self) -> None:
            self.execution_frozen_pub = Publisher()
            self.safety_speed_scale_pub = Publisher()
            self.logged: list[tuple[dict[str, object], bool]] = []

        def _log_state_reason(
            self, payload: dict[str, object], frozen: bool
        ) -> None:
            self.logged.append((payload, frozen))

    monitor = StateHarness()

    monitor._set_state({"status": "replan_requested", "action": "slow", "path_blocked": True})

    assert monitor.current_action == "slow"
    assert monitor.execution_frozen_pub.messages[-1].data is False
    assert monitor.safety_speed_scale_pub.messages[-1].data == 0.9
