#!/usr/bin/env python3
from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass(frozen=True)
class PoseSample:
    stamp: float
    x: float
    y: float
    z: float
    yaw: float


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class TfVelocityEstimator:
    """Estimate map-frame velocity from consecutive TF samples."""

    def __init__(
        self,
        *,
        smoothing_alpha: float,
        max_linear_speed: float,
        max_angular_speed: float,
        reset_gap_sec: float,
    ) -> None:
        self.alpha = min(max(float(smoothing_alpha), 0.0), 1.0)
        self.max_linear_speed = max(float(max_linear_speed), 0.0)
        self.max_angular_speed = max(float(max_angular_speed), 0.0)
        self.reset_gap_sec = max(float(reset_gap_sec), 0.01)
        self.previous: PoseSample | None = None
        self.velocity = (0.0, 0.0, 0.0, 0.0)

    def update(self, sample: PoseSample) -> tuple[float, float, float, float]:
        previous = self.previous
        if previous is None:
            self.previous = sample
            self.velocity = (0.0, 0.0, 0.0, 0.0)
            return self.velocity

        dt = sample.stamp - previous.stamp
        if dt <= 1e-4:
            return self.velocity

        self.previous = sample
        if dt > self.reset_gap_sec:
            self.velocity = (0.0, 0.0, 0.0, 0.0)
            return self.velocity

        raw = (
            (sample.x - previous.x) / dt,
            (sample.y - previous.y) / dt,
            (sample.z - previous.z) / dt,
            normalize_angle(sample.yaw - previous.yaw) / dt,
        )
        linear_speed = math.sqrt(raw[0] ** 2 + raw[1] ** 2 + raw[2] ** 2)
        if (
            not all(math.isfinite(value) for value in raw)
            or linear_speed > self.max_linear_speed
            or abs(raw[3]) > self.max_angular_speed
        ):
            self.velocity = (0.0, 0.0, 0.0, 0.0)
            return self.velocity

        self.velocity = tuple(
            self.alpha * value + (1.0 - self.alpha) * old
            for value, old in zip(raw, self.velocity)
        )
        return self.velocity


class ScanTfPosePublisher(Node):
    """Publish map->base_footprint TF as the SCAN body Odometry input."""

    def __init__(self) -> None:
        super().__init__("scan_tf_pose_publisher")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("output_topic", "/scan/body_pose")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("lookup_timeout_sec", 0.03)
        self.declare_parameter("transform_timeout_sec", 0.5)
        self.declare_parameter("velocity_smoothing_alpha", 0.5)
        self.declare_parameter("velocity_reset_gap_sec", 0.5)
        self.declare_parameter("max_linear_speed", 2.0)
        self.declare_parameter("max_angular_speed", 3.0)

        self.global_frame = str(self.get_parameter("global_frame").value).strip()
        self.robot_frame = str(self.get_parameter("robot_frame").value).strip()
        self.output_topic = str(self.get_parameter("output_topic").value).strip()
        self.lookup_timeout = max(
            float(self.get_parameter("lookup_timeout_sec").value), 0.0
        )
        self.transform_timeout = max(
            float(self.get_parameter("transform_timeout_sec").value), 0.05
        )
        publish_rate = max(float(self.get_parameter("publish_rate_hz").value), 1.0)

        if not self.global_frame or not self.robot_frame or not self.output_topic:
            raise ValueError("global_frame, robot_frame and output_topic must be non-empty")

        self.estimator = TfVelocityEstimator(
            smoothing_alpha=float(
                self.get_parameter("velocity_smoothing_alpha").value
            ),
            max_linear_speed=float(self.get_parameter("max_linear_speed").value),
            max_angular_speed=float(self.get_parameter("max_angular_speed").value),
            reset_gap_sec=float(self.get_parameter("velocity_reset_gap_sec").value),
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(Odometry, self.output_topic, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self._publish_pose)
        self.last_warning_at = -math.inf
        self.ready_logged = False

        self.get_logger().info(
            f"SCAN TF pose source: {self.global_frame} -> {self.robot_frame}, "
            f"output={self.output_topic}, rate={publish_rate:.1f}Hz"
        )

    def _warn_throttled(self, message: str) -> None:
        now = self.get_clock().now().nanoseconds / 1_000_000_000.0
        if now - self.last_warning_at < 2.0:
            return
        self.last_warning_at = now
        self.get_logger().warning(message)

    def _publish_pose(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                Time(),
                timeout=Duration(seconds=self.lookup_timeout),
            )
        except TransformException as exc:
            self._warn_throttled(
                f"SCAN body pose TF unavailable: {self.global_frame} -> "
                f"{self.robot_frame}: {exc}"
            )
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        values = (
            translation.x,
            translation.y,
            translation.z,
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )
        if not all(math.isfinite(value) for value in values):
            self._warn_throttled("SCAN body pose TF contains non-finite values")
            return

        quaternion_norm = math.sqrt(
            rotation.x**2 + rotation.y**2 + rotation.z**2 + rotation.w**2
        )
        if quaternion_norm < 1e-6:
            self._warn_throttled("SCAN body pose TF contains an invalid quaternion")
            return

        stamp = transform.header.stamp
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0
        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1_000_000_000.0
        if stamp_sec <= 0.0:
            stamp = now.to_msg()
            stamp_sec = now_sec
        elif now_sec - stamp_sec > self.transform_timeout:
            self._warn_throttled(
                f"SCAN body pose TF is stale: age={now_sec - stamp_sec:.3f}s"
            )
            return

        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y**2 + rotation.z**2),
        )
        vx, vy, vz, vyaw = self.estimator.update(
            PoseSample(
                stamp=stamp_sec,
                x=float(translation.x),
                y=float(translation.y),
                z=float(translation.z),
                yaw=yaw,
            )
        )

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.global_frame
        odom.child_frame_id = self.robot_frame
        odom.pose.pose.position.x = translation.x
        odom.pose.pose.position.y = translation.y
        odom.pose.pose.position.z = translation.z
        odom.pose.pose.orientation.x = rotation.x / quaternion_norm
        odom.pose.pose.orientation.y = rotation.y / quaternion_norm
        odom.pose.pose.orientation.z = rotation.z / quaternion_norm
        odom.pose.pose.orientation.w = rotation.w / quaternion_norm
        # SCAN expects map-frame velocity in the Odometry twist fields.
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        odom.twist.twist.angular.z = vyaw
        self.publisher.publish(odom)

        if not self.ready_logged:
            self.ready_logged = True
            self.get_logger().info(
                f"SCAN body pose TF ready: {self.global_frame} -> {self.robot_frame}"
            )


def main() -> None:
    rclpy.init()
    node = ScanTfPosePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
