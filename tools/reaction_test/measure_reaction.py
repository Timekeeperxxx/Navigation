#!/usr/bin/env python3
"""避障反应时间离线分析。

用法：
  1. 实测时录 bag（人站在测试区外，导航经过时突然跨入）：
       ros2 bag record -o reaction_test \\
         /lio/cloud_world /grid_map/occupancy_inflate \\
         /nav/obstacle_status /cmd_vel /cmd_vel_safe
  2. 分析（测试区用 map 坐标系下的 XY 包围盒描述）：
       python3 measure_reaction.py reaction_test \\
         --zone-x 3.0 4.0 --zone-y -0.5 0.5

输出从"人进入测试区"起算的分层时间戳：
  t0  人的点云首次出现在测试区（/lio/cloud_world）
  t1  测试区体素首次进入膨胀栅格（/grid_map/occupancy_inflate）
  t2  监控层首次报 blocked/replan（/nav/obstacle_status）
  t3  上游 /cmd_vel 首次归零（SCAN 紧急停生效）
  t4  /cmd_vel_safe 首次归零（最终送往底盘的停止指令）
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys


def read_bag(bag_path: str):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id=""),
        ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    type_map = {
        info.name: info.type for info in reader.get_all_topics_and_types()
    }
    return reader, type_map


def cloud_has_point_in_zone(msg, zone, min_points: int) -> bool:
    from sensor_msgs_py import point_cloud2

    x0, x1, y0, y1, z0, z1 = zone
    count = 0
    for p in point_cloud2.read_points(
        msg, field_names=("x", "y", "z"), skip_nans=True
    ):
        if x0 <= p[0] <= x1 and y0 <= p[1] <= y1 and z0 <= p[2] <= z1:
            count += 1
            if count >= min_points:
                return True
    return False


def twist_is_zero(msg, eps: float = 1e-3) -> bool:
    return (
        abs(msg.linear.x) < eps
        and abs(msg.linear.y) < eps
        and abs(msg.angular.z) < eps
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="bag 目录路径")
    parser.add_argument("--zone-x", nargs=2, type=float, required=True,
                        metavar=("X0", "X1"), help="测试区 X 范围（map 系）")
    parser.add_argument("--zone-y", nargs=2, type=float, required=True,
                        metavar=("Y0", "Y1"), help="测试区 Y 范围（map 系）")
    parser.add_argument("--zone-z", nargs=2, type=float, default=[0.15, 1.2],
                        metavar=("Z0", "Z1"),
                        help="测试区 Z 范围，默认 0.15~1.2（人体）")
    parser.add_argument("--min-points", type=int, default=5,
                        help="判定进入的最少点数（抗飘点），默认 5")
    args = parser.parse_args()

    zone = (
        args.zone_x[0], args.zone_x[1],
        args.zone_y[0], args.zone_y[1],
        args.zone_z[0], args.zone_z[1],
    )

    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader, type_map = read_bag(args.bag)

    needed = {
        "/lio/cloud_world": "t0_person_in_cloud",
        "/grid_map/occupancy_inflate": "t1_voxel_in_grid",
        "/nav/obstacle_status": "t2_monitor_blocked",
        "/cmd_vel": "t3_cmd_vel_zero",
        "/cmd_vel_safe": "t4_cmd_vel_safe_zero",
    }
    missing = [t for t in needed if t not in type_map]
    if missing:
        print(f"bag 中缺少 topic：{missing}", file=sys.stderr)
        print(f"bag 中实际包含：{sorted(type_map)}", file=sys.stderr)
        if "/lio/cloud_world" in missing:
            return 1

    marks: dict[str, float] = {}
    msg_classes = {t: get_message(type_map[t]) for t in needed if t in type_map}
    # /cmd_vel 归零仅在 t0 之后才有意义（起步前本来就是零）。
    moving_seen = {"/cmd_vel": False, "/cmd_vel_safe": False}

    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        if topic not in msg_classes:
            continue
        key = needed[topic]
        t = stamp_ns / 1e9

        if key in marks and topic not in ("/cmd_vel", "/cmd_vel_safe"):
            continue

        msg = deserialize_message(raw, msg_classes[topic])

        if topic in ("/lio/cloud_world", "/grid_map/occupancy_inflate"):
            if key not in marks and cloud_has_point_in_zone(
                msg, zone, args.min_points
            ):
                marks[key] = t
        elif topic == "/nav/obstacle_status":
            if key in marks:
                continue
            try:
                payload = json.loads(msg.data)
            except Exception:
                continue
            if payload.get("status") in (
                "blocked", "replan_requested", "caution"
            ) and "t0_person_in_cloud" in marks:
                marks[key] = t
                marks.setdefault(
                    "t2_status_value", payload.get("status")
                )
        else:  # /cmd_vel or /cmd_vel_safe
            zero = twist_is_zero(msg)
            if not zero:
                moving_seen[topic] = True
            elif (
                moving_seen[topic]
                and "t0_person_in_cloud" in marks
                and key not in marks
                and t >= marks["t0_person_in_cloud"]
            ):
                marks[key] = t

    if "t0_person_in_cloud" not in marks:
        print("测试区内始终没有出现点云——检查 zone 坐标是否正确（map 系）。")
        return 1

    t0 = marks["t0_person_in_cloud"]
    print(f"\n测试区: x[{zone[0]}, {zone[1]}] y[{zone[2]}, {zone[3]}] "
          f"z[{zone[4]}, {zone[5]}]  最少点数: {args.min_points}\n")
    print(f"{'环节':<44}{'绝对时间':>14}{'相对t0':>10}")
    print("-" * 68)
    order = [
        ("t0_person_in_cloud", "人进入点云 /lio/cloud_world"),
        ("t1_voxel_in_grid", "体素进入膨胀栅格 /grid_map/occupancy_inflate"),
        ("t2_monitor_blocked", "监控层报警 /nav/obstacle_status"),
        ("t3_cmd_vel_zero", "上游停止 /cmd_vel（SCAN 层）"),
        ("t4_cmd_vel_safe_zero", "最终停止 /cmd_vel_safe（送底盘）"),
    ]
    for key, label in order:
        if key in marks:
            dt = marks[key] - t0
            print(f"{label:<44}{marks[key]:>14.3f}{dt:>+9.3f}s")
        else:
            print(f"{label:<44}{'未发生':>14}{'—':>10}")
    if "t2_status_value" in marks:
        print(f"\n监控层首个报警状态: {marks['t2_status_value']}")
    if "t4_cmd_vel_safe_zero" in marks:
        total = marks["t4_cmd_vel_safe_zero"] - t0
        print(f"\n>>> 端到端软件反应时间: {total:.3f} s"
              f"（0.22 m/s 下滑行 {total * 0.22:.2f} m，"
              "不含 B2 步态物理停步 ~0.3 s）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
