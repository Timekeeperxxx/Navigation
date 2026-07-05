from __future__ import annotations

import json
import math
import threading
import time
from typing import Any

POINT_CLOUD_MAX_POINTS = 1500
POINT_CLOUD_MIN_INTERVAL_SEC = 0.8


def yaw_to_quaternion(yaw: float) -> dict[str, float]:
    half = yaw * 0.5
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half),
        "w": math.cos(half),
    }


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class RosBridge:
    """测试后端使用的 ROS2 topic 桥接。

    该类只依赖 Navigation 对外公开 topic，不依赖 BotDog 内部模块。
    """

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._started_event = threading.Event()
        self._node: Any | None = None
        self._rclpy: Any | None = None
        self._point_cloud2: Any | None = None
        self._publishers: dict[str, Any] = {}
        self._last_cloud_update_at = 0.0
        self._state: dict[str, Any] = {
            "ros_available": False,
            "started": False,
            "runtime_seen": False,
            "last_runtime_status_at": 0.0,
            "last_command_result_at": 0.0,
            "nav_status": None,
            "runtime_status": None,
            "command_result": None,
            "global_path": {"count": 0, "frame_id": "", "points": []},
            "point_cloud": {
                "topic": "",
                "frame_id": "",
                "count": 0,
                "sample_count": 0,
                "points": [],
                "timestamp": 0.0,
            },
            "robot_pose": None,
            "last_error": None,
        }

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="navigation-test-ros", daemon=True)
        self._thread.start()
        self._started_event.wait(timeout=5.0)

    def stop(self) -> None:
        self._stop_event.set()
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
        if self._rclpy is not None:
            try:
                self._rclpy.shutdown()
            except Exception:
                pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3.0)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            data = dict(self._state)
            data["runtime_seen"] = self.runtime_seen()
            return data

    def runtime_seen(self, max_age_sec: float = 5.0) -> bool:
        with self._lock:
            return bool(
                self._state.get("last_runtime_status_at", 0.0)
                and time.time() - float(self._state["last_runtime_status_at"]) <= max_age_sec
            )

    def publish_runtime_command(self, command: dict[str, Any]) -> None:
        self._publish_json("runtime_command", command)

    def publish_goal(self, x: float, y: float, z: float, yaw: float, frame_id: str = "map") -> None:
        self._require_node()
        now = self._node.get_clock().now().to_msg()

        point_msg = self._make_point_stamped(x, y, z, frame_id, now)
        yaw_msg = self._make_float64(yaw)
        start_msg = self._make_bool(True)

        self._publishers["clicked_point"].publish(point_msg)
        self._publishers["goal_yaw"].publish(yaw_msg)
        self._publishers["nav_start"].publish(start_msg)

    def publish_initialpose(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        frame_id: str = "map",
    ) -> None:
        self._require_node()
        msg = self._make_initialpose(x, y, z, yaw, frame_id)
        self._publishers["initialpose"].publish(msg)

    def publish_cancel(self) -> None:
        self._require_node()
        self._publishers["nav_stop"].publish(self._make_bool(True))
        self._publishers["nav_start"].publish(self._make_bool(False))

    def _run(self) -> None:
        try:
            import rclpy
            from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
            from nav_msgs.msg import Odometry, Path
            from sensor_msgs.msg import PointCloud2
            from sensor_msgs_py import point_cloud2
            from std_msgs.msg import Bool, Float64, String

            self._point_cloud2 = point_cloud2
            self._types = {
                "Bool": Bool,
                "Float64": Float64,
                "Odometry": Odometry,
                "Path": Path,
                "PointCloud2": PointCloud2,
                "PointStamped": PointStamped,
                "PoseWithCovarianceStamped": PoseWithCovarianceStamped,
                "String": String,
            }
            self._rclpy = rclpy
            rclpy.init()
            self._node = rclpy.create_node("navigation_test_app")
            self._create_publishers_and_subscriptions()
            with self._lock:
                self._state.update({"ros_available": True, "started": True, "last_error": None})
            self._started_event.set()

            while not self._stop_event.is_set() and rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.2)
        except Exception as exc:  # pylint: disable=broad-except
            with self._lock:
                self._state.update({"ros_available": False, "started": False, "last_error": str(exc)})
            self._started_event.set()

    def _create_publishers_and_subscriptions(self) -> None:
        String = self._types["String"]
        self._publishers = {
            "clicked_point": self._node.create_publisher(self._types["PointStamped"], "/clicked_point", 10),
            "goal_yaw": self._node.create_publisher(self._types["Float64"], "goal_yaw", 10),
            "nav_start": self._node.create_publisher(self._types["Bool"], "/nav_start", 10),
            "nav_stop": self._node.create_publisher(self._types["Bool"], "/nav_stop", 10),
            "initialpose": self._node.create_publisher(
                self._types["PoseWithCovarianceStamped"],
                "/initialpose",
                10,
            ),
            "runtime_command": self._node.create_publisher(String, "/nav/command_json", 10),
        }
        self._node.create_subscription(String, "/nav_status", self._on_nav_status, 10)
        self._node.create_subscription(String, "/nav/runtime_status", self._on_runtime_status, 10)
        self._node.create_subscription(String, "/nav/command_result", self._on_command_result, 10)
        self._node.create_subscription(self._types["Path"], "/global_path", self._on_global_path, 10)
        self._node.create_subscription(self._types["Odometry"], "/lio/odom", self._on_odom, 10)
        for topic in ("/lio/cloud_world", "/terrain_map", "/nav/local_obstacles"):
            self._node.create_subscription(
                self._types["PointCloud2"],
                topic,
                lambda msg, topic_name=topic: self._on_point_cloud(topic_name, msg),
                5,
            )

    def _on_nav_status(self, msg: Any) -> None:
        self._update_json_state("nav_status", msg.data)

    def _on_runtime_status(self, msg: Any) -> None:
        self._update_json_state("runtime_status", msg.data)
        with self._lock:
            self._state["runtime_seen"] = True
            self._state["last_runtime_status_at"] = time.time()

    def _on_command_result(self, msg: Any) -> None:
        self._update_json_state("command_result", msg.data)
        with self._lock:
            self._state["last_command_result_at"] = time.time()

    def _on_global_path(self, msg: Any) -> None:
        points = []
        for pose_stamped in list(msg.poses)[:20]:
            position = pose_stamped.pose.position
            points.append({"x": position.x, "y": position.y, "z": position.z})
        with self._lock:
            self._state["global_path"] = {
                "count": len(msg.poses),
                "frame_id": msg.header.frame_id,
                "points": points,
                "timestamp": time.time(),
            }

    def _on_odom(self, msg: Any) -> None:
        pose = msg.pose.pose
        orientation = pose.orientation
        with self._lock:
            self._state["robot_pose"] = {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
                "yaw": quaternion_to_yaw(
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                ),
                "frame_id": msg.header.frame_id,
                "timestamp": time.time(),
            }

    def _on_point_cloud(self, topic: str, msg: Any) -> None:
        now = time.time()
        if now - self._last_cloud_update_at < POINT_CLOUD_MIN_INTERVAL_SEC:
            return
        self._last_cloud_update_at = now

        total = int(getattr(msg, "width", 0) or 0) * int(getattr(msg, "height", 0) or 0)
        step = max(total // POINT_CLOUD_MAX_POINTS, 1)
        points = []
        valid_count = 0

        try:
            iterator = self._point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            for index, point in enumerate(iterator):
                valid_count = index + 1
                if index % step != 0:
                    continue
                x, y, z = float(point[0]), float(point[1]), float(point[2])
                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue
                points.append([round(x, 3), round(y, 3), round(z, 3)])
                if len(points) >= POINT_CLOUD_MAX_POINTS:
                    break
        except Exception as exc:  # pylint: disable=broad-except
            with self._lock:
                self._state["point_cloud"] = {
                    "topic": topic,
                    "frame_id": getattr(msg.header, "frame_id", ""),
                    "count": total,
                    "sample_count": 0,
                    "points": [],
                    "timestamp": now,
                    "error": str(exc),
                }
            return

        with self._lock:
            self._state["point_cloud"] = {
                "topic": topic,
                "frame_id": msg.header.frame_id,
                "count": total or valid_count,
                "sample_count": len(points),
                "points": points,
                "timestamp": now,
            }

    def _update_json_state(self, key: str, raw: str) -> None:
        try:
            value = json.loads(raw)
        except json.JSONDecodeError:
            value = {"raw": raw}
        with self._lock:
            self._state[key] = value

    def _publish_json(self, publisher_name: str, payload: dict[str, Any]) -> None:
        self._require_node()
        msg = self._types["String"]()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self._publishers[publisher_name].publish(msg)

    def _make_point_stamped(self, x: float, y: float, z: float, frame_id: str, stamp: Any) -> Any:
        msg = self._types["PointStamped"]()
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = float(z)
        return msg

    def _make_float64(self, value: float) -> Any:
        msg = self._types["Float64"]()
        msg.data = float(value)
        return msg

    def _make_bool(self, value: bool) -> Any:
        msg = self._types["Bool"]()
        msg.data = bool(value)
        return msg

    def _make_initialpose(self, x: float, y: float, z: float, yaw: float, frame_id: str) -> Any:
        msg = self._types["PoseWithCovarianceStamped"]()
        msg.header.frame_id = frame_id
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = float(z)
        q = yaw_to_quaternion(float(yaw))
        msg.pose.pose.orientation.x = q["x"]
        msg.pose.pose.orientation.y = q["y"]
        msg.pose.pose.orientation.z = q["z"]
        msg.pose.pose.orientation.w = q["w"]
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942
        return msg

    def _require_node(self) -> None:
        if self._node is None or not self._state.get("ros_available"):
            raise RuntimeError("ROS2 桥接未就绪，请确认已 source ROS2 环境")
