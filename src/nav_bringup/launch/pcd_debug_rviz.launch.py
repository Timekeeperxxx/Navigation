from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _latest_file(scene_dir: Path, patterns: list[str]) -> str:
    for pattern in patterns:
        matches = sorted(scene_dir.glob(pattern))
        if matches:
            return str(matches[-1])
    return ""


def _pcd_node(name: str, file_name: str, topic: str) -> Node:
    return Node(
        package="nav_bringup",
        executable="pcd_debug_publisher.py",
        name=name,
        output="screen",
        parameters=[
            {
                "file_name": file_name,
                "topic": topic,
                "frame_id": "map",
                "publish_period": 1.0,
            }
        ],
    )


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    scene_dir = Path(LaunchConfiguration("scene_dir").perform(context)).expanduser().resolve()
    map_pcd = LaunchConfiguration("map_pcd").perform(context).strip()
    ground_pcd = LaunchConfiguration("ground_pcd").perform(context).strip()
    base_footprint_fill_pcd = LaunchConfiguration("base_footprint_fill_pcd").perform(context).strip()

    if not map_pcd:
        map_pcd = _latest_file(scene_dir, ["map.pcd"])
    if not ground_pcd:
        ground_pcd = _latest_file(scene_dir, ["ground.pcd", "*_ground.pcd"])
    if not base_footprint_fill_pcd:
        base_footprint_fill_pcd = _latest_file(
            scene_dir,
            [
                "footprint_fill.pcd",
                "fill_footpoint.pcd",
                "*_base_footprint_fill.pcd",
                "*footprint_fill.pcd",
                "*fill_footpoint*.pcd",
            ],
        )

    actions = [
        LogInfo(msg=f"RViz PCD scene_dir: {scene_dir}"),
        LogInfo(msg=f"RViz map_pcd: {map_pcd or '未找到'}"),
        LogInfo(msg=f"RViz ground_pcd: {ground_pcd or '未找到'}"),
        LogInfo(msg=f"RViz base_footprint_fill_pcd: {base_footprint_fill_pcd or '未找到'}"),
    ]

    if map_pcd:
        actions.append(_pcd_node("pcd_debug_map", map_pcd, "/pcd_debug/map"))
    if ground_pcd:
        actions.append(_pcd_node("pcd_debug_ground", ground_pcd, "/pcd_debug/ground"))
    if base_footprint_fill_pcd:
        actions.append(
            _pcd_node(
                "pcd_debug_base_footprint_fill",
                base_footprint_fill_pcd,
                "/pcd_debug/base_footprint_fill",
            )
        )

    actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="pcd_debug_rviz",
            output="screen",
            arguments=[
                "-d",
                PathJoinSubstitution([
                    FindPackageShare("nav_bringup"),
                    "rviz",
                    "pcd_debug.rviz",
                ]),
            ],
            condition=IfCondition(LaunchConfiguration("rviz")),
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "scene_dir",
            default_value="/home/jetson/Project/BOTDOG/MAPS/Scene23_多楼层",
        ),
        DeclareLaunchArgument("map_pcd", default_value=""),
        DeclareLaunchArgument("ground_pcd", default_value=""),
        DeclareLaunchArgument("base_footprint_fill_pcd", default_value=""),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=_launch_setup),
    ])
