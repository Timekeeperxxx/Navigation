#!/usr/bin/env python3
from __future__ import annotations

import struct
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class PcdDebugPublisher(Node):
    """把 PCD 文件按固定 frame 发布为 PointCloud2，供 RViz 调试。"""

    def __init__(self) -> None:
        super().__init__("pcd_debug_publisher")
        self.declare_parameter("file_name", "")
        self.declare_parameter("topic", "/pcd_debug/cloud")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_period", 1.0)

        self.file_name = str(self.get_parameter("file_name").value)
        self.topic = str(self.get_parameter("topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_period = float(self.get_parameter("publish_period").value)

        if not self.file_name:
            raise RuntimeError("缺少 file_name 参数")

        points = self._read_pcd(Path(self.file_name))
        self.message = self._build_message(points)
        self.publisher = self.create_publisher(PointCloud2, self.topic, 10)
        self.timer = self.create_timer(max(self.publish_period, 0.1), self._publish)
        self.get_logger().info(
            f"发布 PCD：{self.file_name} -> {self.topic}, frame={self.frame_id}, points={len(points)}"
        )

    def _publish(self) -> None:
        self.message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.message)

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
            return self._read_binary_points(data[line_end + 1 :], fields, sizes, types, counts, points_count)
        if data_type == "ascii":
            return self._read_ascii_points(data[line_end + 1 :].decode("utf-8", errors="replace"), fields)
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
                intensity = self._unpack_value(body, offset + offsets["intensity"], fields, sizes, types, "intensity")
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

    def _read_ascii_points(self, body: str, fields: list[str]) -> list[tuple[float, float, float, float]]:
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
    node = PcdDebugPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
