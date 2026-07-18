import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
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
    # R_base_lidar = Rz(yaw) * Ry(pitch) * Rx(roll).
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

    mount_text = (
        f"xyz=[{x:.4f},{y:.4f},{z:.4f}]m "
        f"rpy=[{values['lidar_mount_roll_deg']:.3f},"
        f"{values['lidar_mount_pitch_deg']:.3f},"
        f"{values['lidar_mount_yaw_deg']:.3f}]deg"
    )
    return [
        LogInfo(msg=f"Lidar mount calibration (base_footprint->base_link): {mount_text}"),
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
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ]


def generate_launch_description():
    map_dir = LaunchConfiguration("map_dir")
    map_name = LaunchConfiguration("map_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_livox = LaunchConfiguration("launch_livox")
    launch_lio = LaunchConfiguration("launch_lio")
    launch_terrain = LaunchConfiguration("launch_terrain")
    rviz = LaunchConfiguration("rviz")
    publish_base_footprint_tf = LaunchConfiguration("publish_base_footprint_tf")
    lidar_mount_x_m = LaunchConfiguration("lidar_mount_x_m")
    lidar_mount_y_m = LaunchConfiguration("lidar_mount_y_m")
    lidar_mount_z_m = LaunchConfiguration("lidar_mount_z_m")
    lidar_mount_roll_deg = LaunchConfiguration("lidar_mount_roll_deg")
    lidar_mount_pitch_deg = LaunchConfiguration("lidar_mount_pitch_deg")
    lidar_mount_yaw_deg = LaunchConfiguration("lidar_mount_yaw_deg")
    lidar_ip = LaunchConfiguration("lidar_ip")
    host_ip = LaunchConfiguration("host_ip")
    livox_config_path = LaunchConfiguration("livox_config_path")
    lio_voxel_filter_size = LaunchConfiguration("lio_voxel_filter_size")
    map_ds_size = LaunchConfiguration("map_ds_size")
    map_preview_ds_size = LaunchConfiguration("map_preview_ds_size")
    map_preview_publish_interval = LaunchConfiguration("map_preview_publish_interval")
    map_preview_max_points = LaunchConfiguration("map_preview_max_points")
    min_effective_points = LaunchConfiguration("min_effective_points")
    max_frame_translation = LaunchConfiguration("max_frame_translation")
    max_frame_rotation_deg = LaunchConfiguration("max_frame_rotation_deg")
    lidar_time_offset = LaunchConfiguration("lidar_time_offset")
    imu_na = LaunchConfiguration("imu_na")
    imu_ng = LaunchConfiguration("imu_ng")
    imu_nba = LaunchConfiguration("imu_nba")
    imu_nbg = LaunchConfiguration("imu_nbg")
    estimate_gravity = LaunchConfiguration("estimate_gravity")
    use_query_time_undistort = LaunchConfiguration("use_query_time_undistort")
    lidar_imu_roll_deg = LaunchConfiguration("lidar_imu_roll_deg")
    lidar_imu_pitch_deg = LaunchConfiguration("lidar_imu_pitch_deg")
    lidar_imu_yaw_deg = LaunchConfiguration("lidar_imu_yaw_deg")
    loop_closure = LaunchConfiguration("loop_closure")
    loop_search_radius = LaunchConfiguration("loop_search_radius")
    loop_icp_score_threshold = LaunchConfiguration("loop_icp_score_threshold")
    loop_map_ds_size = LaunchConfiguration("loop_map_ds_size")
    lio_config = PathJoinSubstitution([
        FindPackageShare("nav_lio"),
        "config",
        "livox_360.yaml",
    ])
    terrain_config = PathJoinSubstitution([
        FindPackageShare("nav_bringup"),
        "config",
        "terrain_analysis.yaml",
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("nav_bringup"),
        "rviz",
        "mapping_debug.rviz",
    ])

    livox_node = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
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

    lio_node = Node(
        package="nav_lio",
        executable="super_lio_node",
        name="super_lio_node",
        output="screen",
        parameters=[
            lio_config,
            {
                "use_sim_time": use_sim_time,
                "lio.map.save_map": True,
                "lio.map.save_map_dir": map_dir,
                "lio.map.map_name": map_name,
                "lio.map.ds_size": map_ds_size,
                "lio.sensor.voxel_fliter_size": lio_voxel_filter_size,
                "lio.output.map_preview_ds_size": map_preview_ds_size,
                "lio.output.map_preview_publish_interval": map_preview_publish_interval,
                "lio.output.map_preview_max_points": map_preview_max_points,
                "lio.safety.min_effective_points": min_effective_points,
                "lio.safety.max_frame_translation": max_frame_translation,
                "lio.safety.max_frame_rotation_deg": max_frame_rotation_deg,
                "lio.sensor.lidar_time_offset": lidar_time_offset,
                "lio.sensor.imu_na": imu_na,
                "lio.sensor.imu_ng": imu_ng,
                "lio.sensor.imu_nba": imu_nba,
                "lio.sensor.imu_nbg": imu_nbg,
                "lio.sensor.use_query_time_undistort": use_query_time_undistort,
                "lio.kf.estimate_gravity": estimate_gravity,
                "lio.extrinsic.lidar_imu_roll_deg": lidar_imu_roll_deg,
                "lio.extrinsic.lidar_imu_pitch_deg": lidar_imu_pitch_deg,
                "lio.extrinsic.lidar_imu_yaw_deg": lidar_imu_yaw_deg,
                "lio.extrinsic.odom_robo_override": True,
                "lio.extrinsic.odom_robo_x_m": ParameterValue(lidar_mount_x_m, value_type=float),
                "lio.extrinsic.odom_robo_y_m": ParameterValue(lidar_mount_y_m, value_type=float),
                "lio.extrinsic.odom_robo_z_m": ParameterValue(lidar_mount_z_m, value_type=float),
                "lio.extrinsic.odom_robo_roll_deg": ParameterValue(lidar_mount_roll_deg, value_type=float),
                "lio.extrinsic.odom_robo_pitch_deg": ParameterValue(lidar_mount_pitch_deg, value_type=float),
                "lio.extrinsic.odom_robo_yaw_deg": ParameterValue(lidar_mount_yaw_deg, value_type=float),
                "lio.loop.enable": loop_closure,
                "lio.loop.search_radius": loop_search_radius,
                "lio.loop.icp_score_threshold": loop_icp_score_threshold,
                "lio.loop.map_ds_size": loop_map_ds_size,
            },
        ],
        arguments=["--ros-args", "--log-level", "info"],
        condition=IfCondition(launch_lio),
    )

    terrain_analysis = Node(
        package="nav_terrain",
        executable="nav_terrain_analysis",
        name="terrainAnalysis",
        output="screen",
        parameters=[
            terrain_config,
            {
                "use_sim_time": use_sim_time,
            },
        ],
        condition=IfCondition(launch_terrain),
    )

    terrain_saver = Node(
        package="nav_terrain",
        executable="nav_save_terrain_map",
        name="save_terrain_map",
        output="screen",
        parameters=[
            terrain_config,
            {
                "use_sim_time": use_sim_time,
                "map_dir": map_dir,
            },
        ],
        condition=IfCondition(launch_terrain),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="mapping_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4"),
        # Large Livox and PointCloud2 messages must be handed to the FastDDS
        # writer thread instead of blocking the ROS executor during publish().
        SetEnvironmentVariable("RMW_FASTRTPS_PUBLICATION_MODE", "ASYNCHRONOUS"),
        DeclareLaunchArgument("map_dir", default_value="/tmp/navigation_map"),
        DeclareLaunchArgument("map_name", default_value="map.pcd"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("launch_livox", default_value="true"),
        DeclareLaunchArgument("launch_lio", default_value="true"),
        DeclareLaunchArgument("launch_terrain", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("publish_base_footprint_tf", default_value="false"),
        DeclareLaunchArgument("lidar_mount_x_m", default_value="0.0"),
        DeclareLaunchArgument("lidar_mount_y_m", default_value="0.0"),
        DeclareLaunchArgument("lidar_mount_z_m", default_value="0.90"),
        DeclareLaunchArgument("lidar_mount_roll_deg", default_value="0.0"),
        DeclareLaunchArgument("lidar_mount_pitch_deg", default_value="19.48"),
        DeclareLaunchArgument("lidar_mount_yaw_deg", default_value="0.0"),
        DeclareLaunchArgument("lidar_ip", default_value="192.168.123.179"),
        DeclareLaunchArgument("host_ip", default_value=""),
        DeclareLaunchArgument("lio_voxel_filter_size", default_value="0.5"),
        DeclareLaunchArgument("map_ds_size", default_value="0.2"),
        DeclareLaunchArgument("map_preview_ds_size", default_value="0.3"),
        DeclareLaunchArgument("map_preview_publish_interval", default_value="20"),
        DeclareLaunchArgument("map_preview_max_points", default_value="150000"),
        DeclareLaunchArgument("min_effective_points", default_value="20"),
        DeclareLaunchArgument("max_frame_translation", default_value="2.0"),
        DeclareLaunchArgument("max_frame_rotation_deg", default_value="45.0"),
        DeclareLaunchArgument("lidar_time_offset", default_value="0.0"),
        DeclareLaunchArgument("imu_na", default_value="0.03"),
        DeclareLaunchArgument("imu_ng", default_value="0.03"),
        DeclareLaunchArgument("imu_nba", default_value="0.0003"),
        DeclareLaunchArgument("imu_nbg", default_value="0.0003"),
        # Align gravity once from stationary IMU data, then keep the world
        # gravity direction fixed during ground-robot mapping.
        DeclareLaunchArgument("estimate_gravity", default_value="false"),
        DeclareLaunchArgument("use_query_time_undistort", default_value="true"),
        DeclareLaunchArgument("lidar_imu_roll_deg", default_value="0.0"),
        DeclareLaunchArgument("lidar_imu_pitch_deg", default_value="0.0"),
        DeclareLaunchArgument("lidar_imu_yaw_deg", default_value="0.0"),
        DeclareLaunchArgument("loop_closure", default_value="false"),
        DeclareLaunchArgument("loop_search_radius", default_value="5.0"),
        DeclareLaunchArgument("loop_icp_score_threshold", default_value="1.0"),
        DeclareLaunchArgument("loop_map_ds_size", default_value="0.1"),
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
            condition=IfCondition(publish_base_footprint_tf),
        ),
        lio_node,
        terrain_analysis,
        terrain_saver,
        rviz_node,
    ])
