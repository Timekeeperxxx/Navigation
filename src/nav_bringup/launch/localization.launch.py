import math
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _static_tf_from_lidar_mount(context, *args, **kwargs):
    """Publish the exact inverse of T_base_footprint_lidar."""
    del args, kwargs
    names = (
        "lidar_mount_x_m",
        "lidar_mount_y_m",
        "lidar_mount_z_m",
        "lidar_mount_roll_deg",
        "lidar_mount_pitch_deg",
        "lidar_mount_yaw_deg",
    )
    values = {
        name: float(LaunchConfiguration(name).perform(context))
        for name in names
    }
    if not all(math.isfinite(value) for value in values.values()):
        raise RuntimeError("雷达安装标定包含非有限数值")

    x = values["lidar_mount_x_m"]
    y = values["lidar_mount_y_m"]
    z = values["lidar_mount_z_m"]
    roll = math.radians(values["lidar_mount_roll_deg"])
    pitch = math.radians(values["lidar_mount_pitch_deg"])
    yaw = math.radians(values["lidar_mount_yaw_deg"])
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    r00 = cy * cp
    r01 = cy * sp * sr - sy * cr
    r02 = cy * sp * cr + sy * sr
    r10 = sy * cp
    r11 = sy * sp * sr + cy * cr
    r12 = sy * sp * cr - cy * sr
    r20 = -sp
    r21 = cp * sr
    r22 = cp * cr
    inverse_translation = (
        -(r00 * x + r10 * y + r20 * z),
        -(r01 * x + r11 * y + r21 * z),
        -(r02 * x + r12 * y + r22 * z),
    )

    half_roll, half_pitch, half_yaw = roll / 2.0, pitch / 2.0, yaw / 2.0
    chr_, shr = math.cos(half_roll), math.sin(half_roll)
    chp, shp = math.cos(half_pitch), math.sin(half_pitch)
    chy, shy = math.cos(half_yaw), math.sin(half_yaw)
    qx = shr * chp * chy - chr_ * shp * shy
    qy = chr_ * shp * chy + shr * chp * shy
    qz = chr_ * chp * shy - shr * shp * chy
    qw = chr_ * chp * chy + shr * shp * shy
    inverse_quaternion = (-qx, -qy, -qz, qw)

    return [
        LogInfo(
            msg=(
                "Lidar mount calibration (base_footprint->base_link): "
                f"xyz=[{x:.4f},{y:.4f},{z:.4f}]m "
                f"rpy=[{values['lidar_mount_roll_deg']:.3f},"
                f"{values['lidar_mount_pitch_deg']:.3f},"
                f"{values['lidar_mount_yaw_deg']:.3f}]deg"
            )
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_link_to_base_footprint",
            arguments=[
                "--x", format(inverse_translation[0], ".12g"),
                "--y", format(inverse_translation[1], ".12g"),
                "--z", format(inverse_translation[2], ".12g"),
                "--qx", format(inverse_quaternion[0], ".12g"),
                "--qy", format(inverse_quaternion[1], ".12g"),
                "--qz", format(inverse_quaternion[2], ".12g"),
                "--qw", format(inverse_quaternion[3], ".12g"),
                "--frame-id", "base_link",
                "--child-frame-id", "base_footprint",
            ],
            output="screen",
        ),
    ]


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
    lidar_mount_x_m = LaunchConfiguration("lidar_mount_x_m")
    lidar_mount_y_m = LaunchConfiguration("lidar_mount_y_m")
    lidar_mount_z_m = LaunchConfiguration("lidar_mount_z_m")
    lidar_mount_roll_deg = LaunchConfiguration("lidar_mount_roll_deg")
    lidar_mount_pitch_deg = LaunchConfiguration("lidar_mount_pitch_deg")
    lidar_mount_yaw_deg = LaunchConfiguration("lidar_mount_yaw_deg")

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
                "lio.extrinsic.odom_robo_override": True,
                "lio.extrinsic.odom_robo_x_m": ParameterValue(lidar_mount_x_m, value_type=float),
                "lio.extrinsic.odom_robo_y_m": ParameterValue(lidar_mount_y_m, value_type=float),
                "lio.extrinsic.odom_robo_z_m": ParameterValue(lidar_mount_z_m, value_type=float),
                "lio.extrinsic.odom_robo_roll_deg": ParameterValue(lidar_mount_roll_deg, value_type=float),
                "lio.extrinsic.odom_robo_pitch_deg": ParameterValue(lidar_mount_pitch_deg, value_type=float),
                "lio.extrinsic.odom_robo_yaw_deg": ParameterValue(lidar_mount_yaw_deg, value_type=float),
            },
        ],
        arguments=["--ros-args", "--log-level", "info"],
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
        DeclareLaunchArgument("lidar_mount_x_m", default_value="0.0"),
        DeclareLaunchArgument("lidar_mount_y_m", default_value="0.0"),
        DeclareLaunchArgument("lidar_mount_z_m", default_value="0.90"),
        DeclareLaunchArgument("lidar_mount_roll_deg", default_value="0.0"),
        DeclareLaunchArgument("lidar_mount_pitch_deg", default_value="19.48"),
        DeclareLaunchArgument("lidar_mount_yaw_deg", default_value="0.0"),
        DeclareLaunchArgument(
            "livox_config_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("livox_ros_driver2"),
                "config",
                "MID360_config.json",
            ]),
        ),
        livox_node,
        OpaqueFunction(
            function=_static_tf_from_lidar_mount,
            condition=IfCondition(launch_relocation),
        ),
        relocation_node,
        OpaqueFunction(function=_ground_pcd_setup, condition=IfCondition(rviz)),
        rviz_node,
    ])
