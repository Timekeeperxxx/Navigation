from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_dir = LaunchConfiguration("map_dir")
    map_name = LaunchConfiguration("map_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_livox = LaunchConfiguration("launch_livox")
    launch_lio = LaunchConfiguration("launch_lio")
    launch_terrain = LaunchConfiguration("launch_terrain")
    rviz = LaunchConfiguration("rviz")
    publish_base_footprint_tf = LaunchConfiguration("publish_base_footprint_tf")
    base_footprint_x = LaunchConfiguration("base_footprint_x")
    base_footprint_y = LaunchConfiguration("base_footprint_y")
    base_footprint_z = LaunchConfiguration("base_footprint_z")
    base_footprint_roll = LaunchConfiguration("base_footprint_roll")
    base_footprint_pitch = LaunchConfiguration("base_footprint_pitch")
    base_footprint_yaw = LaunchConfiguration("base_footprint_yaw")
    lidar_ip = LaunchConfiguration("lidar_ip")
    host_ip = LaunchConfiguration("host_ip")
    livox_config_path = LaunchConfiguration("livox_config_path")
    lio_voxel_filter_size = LaunchConfiguration("lio_voxel_filter_size")
    map_ds_size = LaunchConfiguration("map_ds_size")
    map_preview_ds_size = LaunchConfiguration("map_preview_ds_size")
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

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_link_to_base_footprint",
        arguments=[
            base_footprint_x,
            base_footprint_y,
            base_footprint_z,
            base_footprint_roll,
            base_footprint_pitch,
            base_footprint_yaw,
            "base_link",
            "base_footprint",
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(publish_base_footprint_tf),
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
        DeclareLaunchArgument("map_dir", default_value="/tmp/navigation_map"),
        DeclareLaunchArgument("map_name", default_value="map.pcd"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("launch_livox", default_value="true"),
        DeclareLaunchArgument("launch_lio", default_value="true"),
        DeclareLaunchArgument("launch_terrain", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("publish_base_footprint_tf", default_value="false"),
        DeclareLaunchArgument("base_footprint_x", default_value="0"),
        DeclareLaunchArgument("base_footprint_y", default_value="0"),
        DeclareLaunchArgument("base_footprint_z", default_value="-0.90"),
        DeclareLaunchArgument("base_footprint_roll", default_value="0"),
        DeclareLaunchArgument("base_footprint_pitch", default_value="-0.34"),
        DeclareLaunchArgument("base_footprint_yaw", default_value="0"),
        DeclareLaunchArgument("lidar_ip", default_value="192.168.123.179"),
        DeclareLaunchArgument("host_ip", default_value=""),
        DeclareLaunchArgument("lio_voxel_filter_size", default_value="0.5"),
        DeclareLaunchArgument("map_ds_size", default_value="0.2"),
        DeclareLaunchArgument("map_preview_ds_size", default_value="0.3"),
        DeclareLaunchArgument("lidar_time_offset", default_value="0.0"),
        DeclareLaunchArgument("imu_na", default_value="0.03"),
        DeclareLaunchArgument("imu_ng", default_value="0.03"),
        DeclareLaunchArgument("imu_nba", default_value="0.0003"),
        DeclareLaunchArgument("imu_nbg", default_value="0.0003"),
        DeclareLaunchArgument("estimate_gravity", default_value="true"),
        DeclareLaunchArgument("use_query_time_undistort", default_value="false"),
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
        static_tf_node,
        lio_node,
        terrain_analysis,
        terrain_saver,
        rviz_node,
    ])
