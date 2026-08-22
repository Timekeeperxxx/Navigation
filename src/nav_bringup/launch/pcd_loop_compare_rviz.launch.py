from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _resolve_pcd(scene_dir: Path, explicit: str, candidates: list[str]) -> str:
    if explicit.strip():
        return str(Path(explicit).expanduser().resolve())
    for candidate in candidates:
        path = scene_dir / candidate
        if path.is_file():
            return str(path)
    return ""


def _map_publisher(name: str, file_name: str, topic: str, voxel_size) -> Node:
    return Node(
        package="nav_bringup",
        executable="nav_pcd_map_publisher",
        name=name,
        output="screen",
        parameters=[
            {
                "map_dir": file_name,
                "ground_dir": "",
                "planground_dir": "",
                "global_frame": "map",
                "publish_period": 30.0,
                "map_down_sample": ParameterValue(voxel_size, value_type=float),
            }
        ],
        remappings=[("/mapcloud", topic)],
    )


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    scene_dir = Path(LaunchConfiguration("scene_dir").perform(context)).expanduser().resolve()
    raw_pcd = _resolve_pcd(
        scene_dir,
        LaunchConfiguration("raw_pcd").perform(context),
        ["map_raw.pcd"],
    )
    loop_pcd = _resolve_pcd(
        scene_dir,
        LaunchConfiguration("loop_pcd").perform(context),
        ["map_loop.pcd", "map.pcd"],
    )
    voxel_size = LaunchConfiguration("voxel_size")

    actions = [
        LogInfo(msg=f"PCD compare scene: {scene_dir}"),
        LogInfo(msg=f"PCD compare raw (red): {raw_pcd or 'not found'}"),
        LogInfo(msg=f"PCD compare loop (green): {loop_pcd or 'not found'}"),
    ]
    if raw_pcd:
        actions.append(_map_publisher("pcd_compare_raw", raw_pcd, "/pcd_compare/raw", voxel_size))
    if loop_pcd:
        actions.append(_map_publisher("pcd_compare_loop", loop_pcd, "/pcd_compare/loop", voxel_size))

    actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="pcd_loop_compare_rviz",
            output="screen",
            arguments=[
                "-d",
                PathJoinSubstitution(
                    [FindPackageShare("nav_bringup"), "rviz", "pcd_loop_compare.rviz"]
                ),
            ],
            condition=IfCondition(LaunchConfiguration("rviz")),
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("scene_dir", default_value="/home/jetson/Projects/Maps"),
            DeclareLaunchArgument("raw_pcd", default_value=""),
            DeclareLaunchArgument("loop_pcd", default_value=""),
            DeclareLaunchArgument(
                "voxel_size",
                default_value="0.15",
                description="Display-only voxel size in metres; source PCD files are unchanged.",
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
