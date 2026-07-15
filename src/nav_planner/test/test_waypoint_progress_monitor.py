import json
import math
import sys
from pathlib import Path

import pytest
import rclpy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64, String


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "script"))

from waypoint_progress_monitor import WaypointProgressMonitor  # noqa: E402


@pytest.fixture
def monitor():
    if not rclpy.ok():
        rclpy.init()
    node = WaypointProgressMonitor()
    try:
        yield node
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_repeated_yaw_point_delivery_keeps_final_yaw(monitor):
    """Every repeated yaw + point pair must retain the requested heading."""
    expected_yaw = -0.73

    for _ in range(3):
        yaw = Float64()
        yaw.data = expected_yaw
        monitor._on_goal_yaw(yaw)

        point = PointStamped()
        point.header.frame_id = "map"
        point.point.x = 1.0
        point.point.y = -2.0
        monitor._on_clicked_point(point)

        assert monitor.active_goal is point
        assert math.isclose(monitor.active_goal_yaw, expected_yaw)
        assert monitor.pending_goal_yaw is None


def test_default_completion_tolerances_are_precise(monitor):
    assert monitor.reach_tolerance_xy == pytest.approx(0.12)
    assert monitor.reach_tolerance_yaw == pytest.approx(0.10)


def _goal(x=1.0, y=-2.0, z=0.3):
    point = PointStamped()
    point.header.frame_id = "map"
    point.point.x = x
    point.point.y = y
    point.point.z = z
    return point


def _context(*, is_final, x=1.0, y=-2.0, z=0.3):
    return String(data=json.dumps({
        "active": True,
        "waypoint_index": 1 if is_final else 0,
        "waypoint_count": 2,
        "is_final": is_final,
        "goal": {"frame_id": "map", "x": x, "y": y, "z": z, "yaw": 1.2},
    }))


def test_intermediate_task_waypoint_does_not_require_yaw(monitor):
    monitor._on_waypoint_context(_context(is_final=False))
    monitor._on_clicked_point(_goal())

    assert monitor.active_task_waypoint_context is not None
    assert monitor._goal_requires_yaw() is False


def test_final_task_waypoint_still_requires_yaw(monitor):
    # Also cover the reverse DDS delivery order: point first, context second.
    monitor._on_clicked_point(_goal())
    monitor._on_waypoint_context(_context(is_final=True))

    assert monitor.active_task_waypoint_context is not None
    assert monitor._goal_requires_yaw() is True


def test_context_for_a_different_goal_cannot_change_completion_rule(monitor):
    monitor._on_waypoint_context(_context(is_final=False, x=9.0))
    monitor._on_clicked_point(_goal())

    assert monitor.active_task_waypoint_context is None
    assert monitor._goal_requires_yaw() is True
