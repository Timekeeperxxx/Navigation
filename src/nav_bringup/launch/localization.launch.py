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


def _ground_pcd_setup(context, *args, **kwargs):
    del args, kwargs
    ground_pcd = LaunchConfiguration("ground_pcd").perform(context).strip()
    scene_dir_arg = LaunchConfiguration("scene_dir").perform(context).strip()
    nav_lio_map_dir = LaunchConfiguration("nav_lio_map_dir").perform(context).strip()

    scene_dir = Path(scene_dir_arg or nav_lio_map_dir).expanduser().resolve()
    if not ground_pcd and scene_dir.exists():
        ground_pcd = _latest_file(scene_dir, ["ground.pcd", "*_ground.pcd"])

    actions = [LogInfo(msg=f"Relocation RViz ground_pcd: {ground_pcd or '未找到'}")]
    if ground_pcd:
        actions.append(
            Node(
                package="nav_bringup",
                executable="pcd_debug_publisher.py",
                name="relocation_ground_publisher",
                output="screen",
                parameters=[
                    {
                        "file_name": ground_pcd,
                        "topic": "/relocation/ground_map",
                        "frame_id": "map",
                        "publish_period": 1.0,
                    }
                ],
            )
        )
    return actions


def generate_launch_description():
    launch_livox = LaunchConfiguration("launch_livox")
    launch_relocation = LaunchConfiguration("launch_relocation")
    nav_lio_map_dir = LaunchConfiguration("nav_lio_map_dir")
    map_name = LaunchConfiguration("map_name")
    rviz = LaunchConfiguration("rviz")
    host_ip = LaunchConfiguration("host_ip")
    livox_config_path = LaunchConfiguration("livox_config_path")

    livox_node = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[{
            "xfer_format": 1,
            "multi_topic": 0,
            "data_src": 0,
            "publish_freq": 10.0,
            "output_data_type": 0,
            "frame_id": "livox_frame",
            "lvx_file_path": "/tmp/livox_test.lvx",
            "user_config_path": livox_config_path,
            "cmdline_input_bd_code": "livox0000000001",
            "user_ip": host_ip,
        }],
        condition=IfCondition(launch_livox),
    )

    relocation_node = Node(
        package="nav_lio",
        executable="relocation_node",
        name="relocation_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("nav_lio"),
                "config",
                "relocation.yaml",
            ]),
            {
                "lio.map.save_map_dir": nav_lio_map_dir,
                "lio.map.map_name": map_name,
            },
        ],
        arguments=["--ros-args", "--log-level", "info"],
        condition=IfCondition(launch_relocation),
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_link_to_base_footprint",
        arguments=["0", "0", "-0.90", "0", "-0.34", "0", "base_link", "base_footprint"],
        output="screen",
        condition=IfCondition(launch_relocation),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="navigation_relocation_rviz",
        arguments=[
            "-d",
            PathJoinSubstitution([
                FindPackageShare("nav_lio"),
                "rviz",
                "relocation.rviz",
            ]),
        ],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("scene_dir", default_value=""),
        DeclareLaunchArgument("nav_lio_map_dir", default_value="map"),
        DeclareLaunchArgument("map_name", default_value="map.pcd"),
        DeclareLaunchArgument("ground_pcd", default_value=""),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("launch_livox", default_value="true"),
        DeclareLaunchArgument("launch_relocation", default_value="true"),
        DeclareLaunchArgument("lidar_ip", default_value="192.168.123.179"),
        DeclareLaunchArgument("host_ip", default_value=""),
        DeclareLaunchArgument(
            "livox_config_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("livox_ros_driver2"),
                "config",
                "MID360_config.json",
            ]),
        ),
        livox_node,
        static_tf_node,
        relocation_node,
        OpaqueFunction(function=_ground_pcd_setup, condition=IfCondition(rviz)),
        rviz_node,
    ])
