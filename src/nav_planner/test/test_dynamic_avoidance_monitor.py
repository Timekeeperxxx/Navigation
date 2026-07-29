from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

from geometry_msgs.msg import PoseStamped


SCRIPT_PATH = Path(__file__).parents[1] / "script" / "dynamic_avoidance_monitor.py"
SPEC = importlib.util.spec_from_file_location("dynamic_avoidance_monitor_under_test", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)

Point3 = MODULE.Point3
PathWindow = MODULE.PathWindow
ObstacleCheck = MODULE.ObstacleCheck
GroundSupportIndex = MODULE.GroundSupportIndex


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
    _check_ground_support = MODULE.DynamicAvoidanceMonitor._check_ground_support
    _check_obstacles = MODULE.DynamicAvoidanceMonitor._check_obstacles

    def __init__(self, obstacles: list[Point3]) -> None:
        self.obstacles = obstacles


class PathParsingHarness:
    global_frame = "map"

    _quaternion_yaw = staticmethod(
        MODULE.DynamicAvoidanceMonitor._quaternion_yaw
    )
    _transform_point = MODULE.DynamicAvoidanceMonitor._transform_point
    _points_from_path = MODULE.DynamicAvoidanceMonitor._points_from_path


class GroundMonitorHarness(MonitorHarness):
    require_ground_support = True
    active_path_source = "scan_execution_path"
    ground_body_height = 0.32
    ground_support_xy_tolerance = 0.14
    ground_support_z_tolerance = 0.20
    ground_footprint_radius = 0.27
    ground_footprint_probe_margin = 0.19
    ground_perimeter_samples = 16
    ground_radial_samples = 2
    ground_outer_ring_max_missing_per_circle = 3
    ground_error = ""

    def __init__(self, ground: "np.ndarray") -> None:
        super().__init__([])
        self.ground_support = GroundSupportIndex(
            self.ground_support_xy_tolerance,
            self.ground_support_z_tolerance,
        )
        if len(ground):
            assert self.ground_support.set_points(ground)


class DecisionHarness:
    enforce_path_blocking = True
    replan_blocked_duration = 2.0
    blocked_clear_duration = 2.0
    ground_replan_blocked_duration = 0.0
    ground_replan_clear_duration = 0.4
    transient_clear_duration = 0.4
    stop_distance = 1.2
    slow_distance = 1.8
    slow_nearby_obstacles = False
    active_path_source = "scan_execution_path"
    execution_path_generation_time = 10.0

    _decide_status = MODULE.DynamicAvoidanceMonitor._decide_status
    _distance_2d = MODULE.DynamicAvoidanceMonitor._distance_2d
    _normalize_angle = staticmethod(
        MODULE.DynamicAvoidanceMonitor._normalize_angle
    )
    _is_in_place_rotation_path = (
        MODULE.DynamicAvoidanceMonitor._is_in_place_rotation_path
    )
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
        self.blocked_kind = None
        self.blocked_execution_generation_time = None
        self.replan_confirmation_required = False
        self.replan_active = False
        self.replan_events: list[bool] = []
        self.terminal_yaw_denied_generation = None
        self.terminal_yaw_denied_reason = ""

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


class _Publisher:
    def __init__(self) -> None:
        self.messages: list[object] = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


class _Logger:
    def __init__(self) -> None:
        self.infos: list[str] = []
        self.warnings: list[str] = []

    def info(self, message: str) -> None:
        self.infos.append(message)

    def warn(self, message: str) -> None:
        self.warnings.append(message)


class GoalSwitchHarness:
    _distance_3d = MODULE.DynamicAvoidanceMonitor._distance_3d
    _execution_path_matches_global = (
        MODULE.DynamicAvoidanceMonitor._execution_path_matches_global
    )
    _execution_path_has_motion = (
        MODULE.DynamicAvoidanceMonitor._execution_path_has_motion
    )
    _release_goal_switch_hold_if_ready = (
        MODULE.DynamicAvoidanceMonitor._release_goal_switch_hold_if_ready
    )
    _on_planning_status = MODULE.DynamicAvoidanceMonitor._on_planning_status
    _on_cmd_vel = MODULE.DynamicAvoidanceMonitor._on_cmd_vel
    _on_nav_start = MODULE.DynamicAvoidanceMonitor._on_nav_start
    _on_nav_stop = MODULE.DynamicAvoidanceMonitor._on_nav_stop

    def __init__(self) -> None:
        self.now = 10.0
        self.hold_during_goal_switch = True
        self.enable_cmd_vel_filter = True
        self.require_nav_start = False
        self.navigation_enabled = True
        self.goal_switch_hold = False
        self.goal_switch_generation = None
        self.goal_switch_status = "idle"
        self.goal_switch_message = ""
        self.goal_switch_started_at = None
        self.goal_switch_candidate_global_path_time = None
        self.goal_switch_ready_global_path_time = None
        self.last_execution_path_time = None
        self.execution_path_generation_time = None
        self.execution_path_points: list[Point3] = []
        self.path_points: list[Point3] = []
        self.scan_execution_path_global_tolerance = 1.0
        self.cmd_vel_safe_pub = _Publisher()
        self.logger = _Logger()
        self.current_action = "clear"

    def _now_sec(self) -> float:
        return self.now

    def get_logger(self) -> _Logger:
        return self.logger

    def _filter_twist(self, msg):
        return msg


class TwistFilterHarness:
    _filter_twist = MODULE.DynamicAvoidanceMonitor._filter_twist

    def __init__(self, action: str) -> None:
        self.current_action = action
        self.slow_speed_scale = 0.70
        self.clearance_escape_speed_scale = 0.60
        self.minimum_nonzero_planar_speed = 0.15
        self.minimum_in_place_yaw_speed = 0.20


ROBOT = Point3(0.0, 0.0, 0.0)
PATH = [Point3(0.0, 0.0, 0.30), Point3(2.0, 0.0, 0.30)]


def ground_plane(
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    z: float,
    spacing: float = 0.10,
):
    import numpy as np

    xs = np.arange(min_x, max_x + spacing * 0.5, spacing)
    ys = np.arange(min_y, max_y + spacing * 0.5, spacing)
    xx, yy = np.meshgrid(xs, ys, indexing="ij")
    return np.column_stack(
        [xx.reshape(-1), yy.reshape(-1), np.full(xx.size, z)]
    )


def exact_circle_ground(
    centers: list[Point3],
    missing_outer_samples: list[list[int]],
):
    import numpy as np

    radius = (
        GroundMonitorHarness.ground_footprint_radius
        + GroundMonitorHarness.ground_footprint_probe_margin
    )
    points: list[tuple[float, float, float]] = []
    for circle_index, center in enumerate(centers):
        ground_z = center.z - GroundMonitorHarness.ground_body_height
        points.append((center.x, center.y, ground_z))
        for ring in range(
            1, GroundMonitorHarness.ground_radial_samples + 1
        ):
            ring_radius = (
                radius
                * ring
                / GroundMonitorHarness.ground_radial_samples
            )
            ring_samples = (
                GroundMonitorHarness.ground_perimeter_samples
                if ring == GroundMonitorHarness.ground_radial_samples
                else max(
                    4, GroundMonitorHarness.ground_perimeter_samples // 2
                )
            )
            for sample in range(ring_samples):
                if (
                    ring == GroundMonitorHarness.ground_radial_samples
                    and sample in missing_outer_samples[circle_index]
                ):
                    continue
                angle = 2.0 * math.pi * sample / ring_samples
                points.append(
                    (
                        center.x + ring_radius * math.cos(angle),
                        center.y + ring_radius * math.sin(angle),
                        ground_z,
                    )
                )
    return np.asarray(points, dtype=np.float64)


def _planning_status(
    status: str, generation: int, message: str = ""
):
    return MODULE.String(
        data=(
            f'{{"status":"{status}","generation":{generation},'
            f'"message":"{message}"}}'
        )
    )


def test_new_goal_latches_zero_speed_until_matching_execution_path() -> None:
    monitor = GoalSwitchHarness()

    monitor._on_planning_status(_planning_status("queued", 7, "queued"))

    assert monitor.goal_switch_hold is True
    assert monitor.goal_switch_generation == 7
    assert len(monitor.cmd_vel_safe_pub.messages) == 1

    # A path_ready status alone cannot resume the old trajectory. The matching
    # global path may be delivered on another ROS callback after this status.
    monitor._on_planning_status(_planning_status("path_ready", 7))
    assert monitor.goal_switch_hold is True

    monitor.path_points = [
        Point3(0.0, 0.0, 0.30),
        Point3(2.0, 0.0, 0.30),
    ]
    monitor.goal_switch_candidate_global_path_time = 10.2
    monitor.goal_switch_ready_global_path_time = 10.2

    # SCAN's reference-swap emergency spline is fresh but stationary. It must
    # not release the hold.
    monitor.last_execution_path_time = 10.3
    monitor.execution_path_generation_time = 10.3
    monitor.execution_path_points = [
        Point3(0.0, 0.0, 0.30),
        Point3(0.0, 0.0, 0.30),
    ]
    monitor._release_goal_switch_hold_if_ready()
    assert monitor.goal_switch_hold is True

    # A newly generated moving spline that agrees with the new global path is
    # the atomic handoff point.
    monitor.execution_path_points = [
        Point3(0.0, 0.0, 0.30),
        Point3(0.4, 0.0, 0.30),
    ]
    monitor._release_goal_switch_hold_if_ready()
    assert monitor.goal_switch_hold is False
    assert monitor.goal_switch_status == "execution_ready"


def test_planning_failure_keeps_old_trajectory_preempted() -> None:
    monitor = GoalSwitchHarness()
    monitor._on_planning_status(_planning_status("planning", 11))

    monitor._on_planning_status(
        _planning_status("failed", 11, "no supported path")
    )

    assert monitor.goal_switch_hold is True
    assert monitor.goal_switch_status == "failed"
    before = len(monitor.cmd_vel_safe_pub.messages)
    monitor._on_cmd_vel(MODULE.Twist())
    assert len(monitor.cmd_vel_safe_pub.messages) == before + 1
    stopped = monitor.cmd_vel_safe_pub.messages[-1]
    assert stopped.linear.x == 0.0
    assert stopped.angular.z == 0.0


def test_newer_goal_generation_ignores_stale_planner_status() -> None:
    monitor = GoalSwitchHarness()
    monitor._on_planning_status(_planning_status("queued", 20))
    monitor.goal_switch_candidate_global_path_time = 10.1
    monitor.goal_switch_ready_global_path_time = 10.1

    monitor.now = 11.0
    monitor._on_planning_status(_planning_status("queued", 21))
    monitor._on_planning_status(_planning_status("path_ready", 20))

    assert monitor.goal_switch_hold is True
    assert monitor.goal_switch_generation == 21
    assert monitor.goal_switch_candidate_global_path_time is None
    assert monitor.goal_switch_ready_global_path_time is None
    assert monitor.goal_switch_status == "queued"


def test_single_goto_ignores_task_stop_and_new_goal_reenables_after_nav_stop() -> None:
    monitor = GoalSwitchHarness()

    monitor._on_nav_start(MODULE.Bool(data=False))
    assert monitor.navigation_enabled is True

    monitor._on_nav_stop(MODULE.Bool(data=True))
    assert monitor.navigation_enabled is False

    monitor._on_planning_status(_planning_status("queued", 22))
    assert monitor.navigation_enabled is True


def test_legacy_nav_start_gate_still_honors_false_when_explicitly_enabled() -> None:
    monitor = GoalSwitchHarness()
    monitor.require_nav_start = True

    monitor._on_nav_start(MODULE.Bool(data=False))

    assert monitor.navigation_enabled is False
    assert len(monitor.cmd_vel_safe_pub.messages) == 1


def test_b2_slow_filter_keeps_nonzero_translation_above_gait_deadzone() -> None:
    monitor = TwistFilterHarness("slow")
    command = MODULE.Twist()
    command.linear.x = 0.15

    filtered = monitor._filter_twist(command)

    assert math.isclose(filtered.linear.x, 0.15)
    assert math.isclose(filtered.linear.y, 0.0)


def test_b2_escape_filter_keeps_in_place_yaw_above_gait_deadzone() -> None:
    monitor = TwistFilterHarness("escape")
    command = MODULE.Twist()
    command.angular.z = -0.20

    filtered = monitor._filter_twist(command)

    assert math.isclose(filtered.angular.z, -0.20)


def test_b2_zero_speed_scale_never_creates_motion() -> None:
    monitor = TwistFilterHarness("escape")
    monitor.clearance_escape_speed_scale = 0.0
    command = MODULE.Twist()
    command.angular.z = -0.20

    filtered = monitor._filter_twist(command)

    assert filtered.angular.z == 0.0


def test_b2_stop_filter_remains_exactly_zero() -> None:
    monitor = TwistFilterHarness("stop")
    command = MODULE.Twist()
    command.linear.x = 0.20
    command.angular.z = 0.30

    filtered = monitor._filter_twist(command)

    assert filtered.linear.x == 0.0
    assert filtered.angular.z == 0.0


def test_dense_same_floor_ground_supports_complete_execution_footprint() -> None:
    monitor = GroundMonitorHarness(
        ground_plane(-2.0, 3.0, -2.0, 2.0, -0.02)
    )

    result = monitor._check_obstacles(ROBOT, PATH)

    assert result.path_blocked is False
    assert result.ground_supported is True
    assert result.ground_probe_count > 0


def test_three_missing_outer_probes_per_circle_remain_supported() -> None:
    centers = [Point3(-1.0, 0.0, 0.30), Point3(1.0, 0.0, 0.30)]
    ground = exact_circle_ground(
        centers, [[0, 5, 11], [2, 7, 13]]
    )
    monitor = GroundMonitorHarness(ground)
    monitor.ground_support = GroundSupportIndex(0.01, 0.20)
    assert monitor.ground_support.set_points(ground)

    result = monitor._check_ground_support(
        ROBOT, [[centers[0]], [centers[1]]]
    )

    assert result.supported is True
    assert result.unsupported_probe_count == 6


def test_four_missing_outer_probes_on_one_circle_are_rejected() -> None:
    centers = [Point3(-1.0, 0.0, 0.30), Point3(1.0, 0.0, 0.30)]
    ground = exact_circle_ground(centers, [[0, 4, 8, 12], []])
    monitor = GroundMonitorHarness(ground)
    monitor.ground_support = GroundSupportIndex(0.01, 0.20)
    assert monitor.ground_support.set_points(ground)

    result = monitor._check_ground_support(
        ROBOT, [[centers[0]], [centers[1]]]
    )

    assert result.supported is False
    assert result.unsupported_probe_count == 4


def test_execution_path_crossing_ground_hole_is_blocked() -> None:
    import numpy as np

    ground = np.vstack(
        [
            ground_plane(-2.0, 0.45, -2.0, 2.0, -0.02),
            ground_plane(1.55, 3.0, -2.0, 2.0, -0.02),
        ]
    )
    monitor = GroundMonitorHarness(ground)

    result = monitor._check_obstacles(ROBOT, PATH)

    assert result.path_blocked is True
    assert result.ground_supported is False
    assert result.unsupported_ground_probe_count > 0
    assert result.clearance_escape is False


def test_other_floor_cannot_fill_current_floor_hole() -> None:
    import numpy as np

    lower_floor = ground_plane(-2.0, 3.0, -2.0, 2.0, -3.0)
    upper_with_hole = np.vstack(
        [
            ground_plane(-2.0, 0.45, -2.0, 2.0, -0.02),
            ground_plane(1.55, 3.0, -2.0, 2.0, -0.02),
        ]
    )
    monitor = GroundMonitorHarness(
        np.vstack([lower_floor, upper_with_hole])
    )

    result = monitor._check_obstacles(ROBOT, PATH)

    assert result.ground_supported is False


def test_ground_xy_and_z_tolerances_match_cpp_rectangular_rule() -> None:
    import numpy as np

    index = GroundSupportIndex(0.14, 0.20)
    assert index.set_points(np.asarray([[0.0, 0.0, 0.0]]))

    supported = index.supported_mask(
        np.asarray(
            [
                [0.13, 0.0, 0.15],
                [0.141, 0.0, 0.0],
                [0.0, 0.0, 0.201],
            ]
        )
    )

    assert supported.tolist() == [True, False, False]


def test_five_millimeter_ground_boundary_allowance_is_bounded() -> None:
    import numpy as np

    index = GroundSupportIndex(0.155, 0.20)
    assert index.set_points(np.asarray([[0.0, 0.0, 0.0]]))

    supported = index.supported_mask(
        np.asarray(
            [
                [0.150394, 0.0, 0.0],
                [0.1551, 0.0, 0.0],
            ]
        )
    )

    assert supported.tolist() == [True, False]


def test_global_path_uses_ground_height_without_body_offset() -> None:
    monitor = GroundMonitorHarness(
        ground_plane(-2.0, 3.0, -2.0, 2.0, 0.0)
    )
    monitor.active_path_source = "global_path"
    global_path = [Point3(0.0, 0.0, 0.0), Point3(2.0, 0.0, 0.0)]

    result = monitor._check_obstacles(ROBOT, global_path)

    assert result.ground_supported is True


def test_missing_ground_index_fails_closed() -> None:
    import numpy as np

    monitor = GroundMonitorHarness(np.empty((0, 3)))

    result = monitor._check_obstacles(ROBOT, PATH)

    assert result.path_blocked is True
    assert result.ground_supported is False
    assert result.nearest_blocker_distance == 0.0
    assert result.ground_error


def test_invalidated_ground_index_cannot_keep_using_old_floor() -> None:
    ground = ground_plane(-2.0, 3.0, -2.0, 2.0, -0.02)
    index = GroundSupportIndex(0.14, 0.20)
    assert index.set_points(ground)
    assert index.ready

    index.invalidate("new map has wrong frame")

    assert index.ready is False
    assert index.point_count == 0
    assert index.error == "new map has wrong frame"


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


def test_execution_path_orientation_is_parsed_but_global_path_ignores_it() -> None:
    monitor = PathParsingHarness()
    path = MODULE.Path()
    path.header.frame_id = "map"
    pose = PoseStamped()
    pose.pose.orientation.z = math.sin(math.pi / 4.0)
    pose.pose.orientation.w = math.cos(math.pi / 4.0)
    path.poses.append(pose)

    execution_points = monitor._points_from_path(
        path, include_pose_yaw=True
    )
    global_points = monitor._points_from_path(
        path, include_pose_yaw=False
    )

    assert math.isclose(execution_points[0].yaw, math.pi / 2.0)
    assert global_points[0].yaw is None


def test_complete_explicit_yaw_is_not_overwritten_by_path_tangent() -> None:
    monitor = MonitorHarness([])
    fixed_sideways_yaw = math.pi / 2.0
    body_path = [
        Point3(0.0, 0.0, 0.30, fixed_sideways_yaw),
        Point3(1.0, 0.0, 0.30, fixed_sideways_yaw),
    ]

    front_sweep, rear_sweep = monitor._footprint_sweep_paths(
        body_path, initial_yaw=0.0
    )

    assert all(
        math.isclose(point.x, body_path[index].x, abs_tol=1e-9)
        for index, point in (
            (0, front_sweep[0]),
            (1, front_sweep[-1]),
            (0, rear_sweep[0]),
            (1, rear_sweep[-1]),
        )
    )
    assert all(
        math.isclose(point.y, offset, abs_tol=1e-9)
        for sweep, offset in zip(
            (front_sweep, rear_sweep), monitor.footprint_center_offsets
        )
        for point in sweep
    )


def test_in_place_yaw_sweep_checks_intermediate_double_circle_attitudes() -> None:
    # This obstacle lies on the rear-circle centre at 45 degrees. It is away
    # from both the 0- and 90-degree endpoints, so endpoint-only checking would
    # miss it.
    monitor = MonitorHarness([Point3(-0.445, -0.445, 0.30)])
    monitor.robot_self_clear_radius = 0.10
    yaw_sweep = [
        Point3(0.0, 0.0, 0.30, 0.0),
        Point3(0.0, 0.0, 0.30, math.pi / 2.0),
    ]

    result = monitor._check_obstacles(ROBOT, yaw_sweep)

    assert result.path_blocked is True
    assert result.nearest_path_distance is not None
    assert result.nearest_path_distance < 0.01


def test_missing_execution_orientation_keeps_tangent_yaw_fallback() -> None:
    monitor = MonitorHarness([])
    body_path = [
        Point3(0.0, 0.0, 0.30),
        Point3(1.0, 0.0, 0.30),
    ]

    yaws = monitor._estimate_path_yaws(
        body_path, initial_yaw=math.pi / 2.0
    )
    sweeps = monitor._footprint_sweep_paths(
        body_path, initial_yaw=math.pi / 2.0
    )

    assert yaws == [math.pi / 2.0, 0.0]
    assert math.isclose(sweeps[0][0].x, 0.0, abs_tol=1e-9)
    assert math.isclose(
        sweeps[0][-1].x,
        1.0 + monitor.footprint_center_offsets[0],
        abs_tol=1e-9,
    )


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


def test_ground_gap_is_immediate_stop_even_when_farther_than_stop_distance() -> None:
    monitor = DecisionHarness()
    window = PathWindow(PATH, PATH, 0, 0.0)
    unsupported = ObstacleCheck(
        True,
        0,
        0,
        None,
        1.6,
        None,
        ground_supported=False,
        nearest_unsupported_ground_distance=1.6,
    )

    result = monitor._decide_status(0.0, ROBOT, window, unsupported)

    assert result["status"] == "replan_requested"
    assert result["action"] == "stop"
    assert monitor.replan_events == [True]


def test_terminal_yaw_ground_failure_stops_without_path_replan() -> None:
    monitor = DecisionHarness()
    yaw_sweep = [
        Point3(0.0, 0.0, 0.30, 0.0),
        Point3(0.0, 0.0, 0.30, math.pi / 2.0),
    ]
    window = PathWindow(yaw_sweep, yaw_sweep, 0, 0.0)
    unsupported = ObstacleCheck(
        True,
        0,
        0,
        None,
        0.4,
        None,
        ground_supported=False,
        nearest_unsupported_ground_distance=0.4,
    )

    first = monitor._decide_status(0.0, ROBOT, window, unsupported)
    persistent = monitor._decide_status(3.0, ROBOT, window, unsupported)

    assert first["status"] == "goal_yaw_blocked"
    assert persistent["status"] == "goal_yaw_blocked"
    assert first["action"] == "stop"
    assert monitor.replan_events == []


def test_terminal_yaw_obstacle_failure_never_enters_replan_timer() -> None:
    monitor = DecisionHarness()
    yaw_sweep = [
        Point3(0.0, 0.0, 0.30, 0.0),
        Point3(0.0, 0.0, 0.30, -math.pi / 2.0),
    ]
    window = PathWindow(yaw_sweep, yaw_sweep, 0, 0.0)
    blocked = ObstacleCheck(True, 1, 0, 0.4, 0.4, 0.0)

    first = monitor._decide_status(0.0, ROBOT, window, blocked)
    persistent = monitor._decide_status(3.0, ROBOT, window, blocked)

    assert first["status"] == "goal_yaw_blocked"
    assert persistent["status"] == "goal_yaw_blocked"
    assert monitor.replan_events == []


def test_skipped_terminal_yaw_remains_visible_without_freezing_xy_goal() -> None:
    monitor = DecisionHarness()
    monitor.terminal_yaw_denied_generation = 10.0
    monitor.terminal_yaw_denied_reason = "ground"
    stationary = [Point3(1.0, 2.0, 0.30, 1.2)]
    window = PathWindow(stationary, stationary, 0, 0.0)
    clear = ObstacleCheck(False, 0, 0, None, None, None)

    result = monitor._decide_status(3.0, ROBOT, window, clear)

    assert result["status"] == "goal_yaw_skipped"
    assert result["action"] == "pass"
    assert result["path_blocked"] is False
    assert monitor.replan_events == []


def test_terminal_yaw_validation_ack_carries_execution_generation() -> None:
    monitor = object.__new__(MODULE.DynamicAvoidanceMonitor)
    monitor.execution_path_generation_time = 12.345
    monitor.final_yaw_validation_pub = _Publisher()

    monitor._publish_final_yaw_validation(True)
    monitor._publish_final_yaw_validation(False)

    assert [
        list(message.data)
        for message in monitor.final_yaw_validation_pub.messages
    ] == [
        [12.345, 1.0],
        [12.345, 0.0],
    ]


def test_ground_gap_cannot_be_disabled_or_bypassed_by_clearance_escape() -> None:
    monitor = DecisionHarness()
    monitor.enforce_path_blocking = False
    window = PathWindow(PATH, PATH, 0, 0.0)
    unsupported_escape = ObstacleCheck(
        True,
        0,
        0,
        None,
        1.6,
        None,
        clearance_escape=True,
        ground_supported=False,
        nearest_unsupported_ground_distance=1.6,
    )

    result = monitor._decide_status(0.0, ROBOT, window, unsupported_escape)

    assert result["status"] == "replan_requested"
    assert result["action"] == "stop"


def test_ground_replan_uses_new_generation_and_short_clear_confirmation() -> None:
    monitor = DecisionHarness()
    window = PathWindow(PATH, PATH, 0, 0.0)
    unsupported = ObstacleCheck(
        True,
        0,
        0,
        None,
        1.6,
        None,
        ground_supported=False,
        nearest_unsupported_ground_distance=1.6,
    )
    clear = ObstacleCheck(False, 0, 0, None, None, None)

    assert (
        monitor._decide_status(0.0, ROBOT, window, unsupported)["status"]
        == "replan_requested"
    )
    assert (
        monitor._decide_status(0.1, ROBOT, window, clear)["status"]
        == "waiting_replan"
    )

    monitor.execution_path_generation_time = 11.0
    assert monitor._decide_status(0.2, ROBOT, window, clear)["status"] == "clearing"
    assert monitor.replan_events == [True, False]
    assert monitor._decide_status(0.59, ROBOT, window, clear)["status"] == "clearing"
    assert monitor._decide_status(0.61, ROBOT, window, clear)["status"] == "clear"


def test_ground_replan_debounce_is_independently_parameterized() -> None:
    monitor = DecisionHarness()
    monitor.ground_replan_blocked_duration = 0.2
    window = PathWindow(PATH, PATH, 0, 0.0)
    unsupported = ObstacleCheck(
        True,
        0,
        0,
        None,
        1.6,
        None,
        ground_supported=False,
        nearest_unsupported_ground_distance=1.6,
    )

    assert monitor._decide_status(0.0, ROBOT, window, unsupported)["status"] == "blocked"
    assert monitor._decide_status(0.19, ROBOT, window, unsupported)["status"] == "blocked"
    assert (
        monitor._decide_status(0.2, ROBOT, window, unsupported)["status"]
        == "replan_requested"
    )
    assert monitor.replan_events == [True]


def test_ground_replan_from_global_fallback_still_waits_for_scan_generation() -> None:
    monitor = DecisionHarness()
    monitor.active_path_source = "global_path"
    monitor.execution_path_generation_time = None
    window = PathWindow(PATH, PATH, 0, 0.0)
    unsupported = ObstacleCheck(
        True,
        0,
        0,
        None,
        1.6,
        None,
        ground_supported=False,
        nearest_unsupported_ground_distance=1.6,
    )
    clear = ObstacleCheck(False, 0, 0, None, None, None)

    monitor._decide_status(0.0, ROBOT, window, unsupported)
    assert (
        monitor._decide_status(0.1, ROBOT, window, clear)["status"]
        == "waiting_replan"
    )

    monitor.active_path_source = "scan_execution_path"
    monitor.execution_path_generation_time = 11.0
    assert monitor._decide_status(0.2, ROBOT, window, clear)["status"] == "clearing"


def test_unavailable_ground_stops_without_futile_replan_requests() -> None:
    monitor = DecisionHarness()
    window = PathWindow(PATH, PATH, 0, 0.0)
    unavailable = ObstacleCheck(
        True,
        0,
        0,
        None,
        0.0,
        None,
        ground_supported=False,
        ground_error="ground 支撑索引尚未就绪",
    )

    result = monitor._decide_status(10.0, ROBOT, window, unavailable)

    assert result["status"] == "ground_unavailable"
    assert result["action"] == "stop"
    assert monitor.replan_events == []


def test_ground_becoming_unavailable_cancels_active_replan_without_retriggering() -> None:
    monitor = DecisionHarness()
    window = PathWindow(PATH, PATH, 0, 0.0)
    unsupported = ObstacleCheck(
        True,
        0,
        0,
        None,
        1.0,
        None,
        ground_supported=False,
        nearest_unsupported_ground_distance=1.0,
    )
    unavailable = ObstacleCheck(
        True,
        0,
        0,
        None,
        0.0,
        None,
        ground_supported=False,
        ground_error="ground 支撑索引尚未就绪",
    )

    monitor._decide_status(0.0, ROBOT, window, unsupported)
    first = monitor._decide_status(0.1, ROBOT, window, unavailable)
    second = monitor._decide_status(0.2, ROBOT, window, unavailable)

    assert first["status"] == "ground_unavailable"
    assert second["status"] == "ground_unavailable"
    assert monitor.replan_events == [True, False]


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
