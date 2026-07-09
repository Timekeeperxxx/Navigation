#!/usr/bin/env python3
from __future__ import annotations

import struct
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


class NavPcdMapPublisher(Node):
    """发布规划静态地图、ground 和 planground 点云。"""

    def __init__(self) -> None:
        super().__init__("nav_pcd_map_publisher")
        self.declare_parameter("map_dir", "")
        self.declare_parameter("ground_dir", "")
        self.declare_parameter("planground_dir", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_period", 1.0)
        self.declare_parameter("map_down_sample", 0.0)
        self.declare_parameter("ground_down_sample", 0.0)
        self.declare_parameter("planground_down_sample", 0.0)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_period = float(self.get_parameter("publish_period").value)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.entries: list[tuple[str, PointCloud2, object]] = []
        for param_name, topic, downsample_param in (
            ("map_dir", "/mapcloud", "map_down_sample"),
            ("ground_dir", "/mapground", "ground_down_sample"),
            ("planground_dir", "/planground", "planground_down_sample"),
        ):
            file_name = str(self.get_parameter(param_name).value).strip()
            if not file_name:
                self.get_logger().warn(f"{param_name} 为空，跳过 {topic}")
                continue

            points = self._read_pcd(Path(file_name))
            raw_count = len(points)
            leaf_size = float(self.get_parameter(downsample_param).value)
            points = self._voxel_downsample(points, leaf_size)
            message = self._build_message(points)
            publisher = self.create_publisher(PointCloud2, topic, qos)
            self.entries.append((topic, message, publisher))
            self.get_logger().info(
                f"发布 {param_name}: {file_name} -> {topic}, raw_points={raw_count}, "
                f"points={len(points)}, down_sample={leaf_size:.3f}"
            )

        if not self.entries:
            raise RuntimeError("没有可发布的 PCD 文件")

        self.timer = self.create_timer(max(self.publish_period, 0.2), self._publish)
        self._publish()

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for _, message, publisher in self.entries:
            message.header.stamp = stamp
            publisher.publish(message)

    def _read_pcd(self, path: Path) -> list[tuple[float, float, float, float]]:
        data = path.read_bytes()
        data_offset = data.find(b"DATA ")
        if data_offset < 0:
            raise RuntimeError(f"PCD 缺少 DATA header：{path}")
        line_end = data.find(b"\n", data_offset)
        if line_end < 0:
            raise RuntimeError(f"PCD DATA header 不完整：{path}")

        header_text = data[: line_end + 1].decode("latin1")
        header: dict[str, list[str]] = {}
        for raw_line in header_text.splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            header[parts[0].upper()] = parts[1:]

        fields = header.get("FIELDS", [])
        sizes = [int(value) for value in header.get("SIZE", [])]
        types = header.get("TYPE", [])
        counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
        points_count = int(header.get("POINTS", header.get("WIDTH", ["0"]))[0])
        data_type = header.get("DATA", [""])[0].lower()
        if not {"x", "y", "z"}.issubset(set(fields)):
            raise RuntimeError(f"PCD 缺少 x/y/z 字段：{path}")

        if data_type == "binary":
            return self._read_binary_points(
                data[line_end + 1 :], fields, sizes, types, counts, points_count
            )
        if data_type == "ascii":
            return self._read_ascii_points(
                data[line_end + 1 :].decode("utf-8", errors="replace"), fields
            )
        raise RuntimeError(f"不支持的 PCD DATA 类型：{data_type}")

    def _read_binary_points(
        self,
        body: bytes,
        fields: list[str],
        sizes: list[int],
        types: list[str],
        counts: list[int],
        points_count: int,
    ) -> list[tuple[float, float, float, float]]:
        offsets: dict[str, int] = {}
        point_step = 0
        for field, size, count in zip(fields, sizes, counts):
            offsets[field] = point_step
            point_step += size * count

        points: list[tuple[float, float, float, float]] = []
        for index in range(points_count):
            offset = index * point_step
            if offset + point_step > len(body):
                break
            x = self._unpack_value(body, offset + offsets["x"], fields, sizes, types, "x")
            y = self._unpack_value(body, offset + offsets["y"], fields, sizes, types, "y")
            z = self._unpack_value(body, offset + offsets["z"], fields, sizes, types, "z")
            intensity = 0.0
            if "intensity" in offsets:
                intensity = self._unpack_value(
                    body, offset + offsets["intensity"], fields, sizes, types, "intensity"
                )
            points.append((x, y, z, intensity))
        return points

    def _unpack_value(
        self,
        body: bytes,
        offset: int,
        fields: list[str],
        sizes: list[int],
        types: list[str],
        field: str,
    ) -> float:
        idx = fields.index(field)
        size = sizes[idx]
        value_type = types[idx].upper()
        if value_type == "F" and size == 4:
            return float(struct.unpack_from("<f", body, offset)[0])
        if value_type == "F" and size == 8:
            return float(struct.unpack_from("<d", body, offset)[0])
        if value_type == "I" and size == 4:
            return float(struct.unpack_from("<i", body, offset)[0])
        if value_type == "U" and size == 4:
            return float(struct.unpack_from("<I", body, offset)[0])
        raise RuntimeError(f"不支持的 PCD 字段类型：{field} {value_type}{size}")

    def _read_ascii_points(
        self, body: str, fields: list[str]
    ) -> list[tuple[float, float, float, float]]:
        x_idx = fields.index("x")
        y_idx = fields.index("y")
        z_idx = fields.index("z")
        intensity_idx = fields.index("intensity") if "intensity" in fields else -1
        points: list[tuple[float, float, float, float]] = []
        for line in body.splitlines():
            if not line.strip():
                continue
            values = line.split()
            intensity = float(values[intensity_idx]) if intensity_idx >= 0 else 0.0
            points.append((float(values[x_idx]), float(values[y_idx]), float(values[z_idx]), intensity))
        return points

    def _voxel_downsample(
        self, points: list[tuple[float, float, float, float]], leaf_size: float
    ) -> list[tuple[float, float, float, float]]:
        if leaf_size <= 0.0 or len(points) <= 1:
            return points

        inv_leaf = 1.0 / leaf_size
        voxels: dict[tuple[int, int, int], list[float]] = {}
        for x, y, z, intensity in points:
            key = (
                math.floor(x * inv_leaf),
                math.floor(y * inv_leaf),
                math.floor(z * inv_leaf),
            )
            bucket = voxels.get(key)
            if bucket is None:
                voxels[key] = [x, y, z, intensity, 1.0]
            else:
                bucket[0] += x
                bucket[1] += y
                bucket[2] += z
                bucket[3] += intensity
                bucket[4] += 1.0

        return [
            (
                bucket[0] / bucket[4],
                bucket[1] / bucket[4],
                bucket[2] / bucket[4],
                bucket[3] / bucket[4],
            )
            for bucket in voxels.values()
        ]

    def _build_message(self, points: list[tuple[float, float, float, float]]) -> PointCloud2:
        message = PointCloud2()
        message.header.frame_id = self.frame_id
        message.height = 1
        message.width = len(points)
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 16
        message.row_step = message.point_step * len(points)
        message.is_dense = True
        message.data = b"".join(struct.pack("<ffff", *point) for point in points)
        return message


def main() -> None:
    rclpy.init()
    node = NavPcdMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
