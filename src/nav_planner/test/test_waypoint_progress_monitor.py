import math
import sys
from pathlib import Path

import pytest
import rclpy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64


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
