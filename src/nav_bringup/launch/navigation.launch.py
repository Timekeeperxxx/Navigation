from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scene_dir = LaunchConfiguration("scene_dir")
    nav_lio_map_dir = LaunchConfiguration("nav_lio_map_dir")
    map_name = LaunchConfiguration("map_name")
    map_pcd = LaunchConfiguration("map_pcd")
    ground_pcd = LaunchConfiguration("ground_pcd")
    planground_pcd = LaunchConfiguration("planground_pcd")
    roi_file_path = LaunchConfiguration("roi_file_path")
    waypoints_file = LaunchConfiguration("waypoints_file")
    rviz = LaunchConfiguration("rviz")
    navigation_rviz = LaunchConfiguration("navigation_rviz")
    launch_relocation = LaunchConfiguration("launch_relocation")
    enable_pcl_publisher = LaunchConfiguration("enable_pcl_publisher")
    enable_polygon_loader = LaunchConfiguration("enable_polygon_loader")
    enable_dwa_client = LaunchConfiguration("enable_dwa_client")
    enable_waypoint_navigator = LaunchConfiguration("enable_waypoint_navigator")
    waypoint_navigator_start_topic = LaunchConfiguration("waypoint_navigator_start_topic")
    enable_waypoint_monitor = LaunchConfiguration("enable_waypoint_monitor")
    enable_dynamic_avoidance = LaunchConfiguration("enable_dynamic_avoidance")
    enable_path_follower = LaunchConfiguration("enable_path_follower")
    enable_obstacle_simulator = LaunchConfiguration("enable_obstacle_simulator")
    enable_scan_planner = LaunchConfiguration("enable_scan_planner")
    enable_scan_controller = LaunchConfiguration("enable_scan_controller")
    enable_scan_path_adapter = LaunchConfiguration("enable_scan_path_adapter")
    enable_scan_tf_pose = LaunchConfiguration("enable_scan_tf_pose")
    scan_planner_config = LaunchConfiguration("scan_planner_config")
    scan_body_pose_topic = LaunchConfiguration("scan_body_pose_topic")
    scan_sensor_pose_topic = LaunchConfiguration("scan_sensor_pose_topic")
    scan_cloud_topic = LaunchConfiguration("scan_cloud_topic")
    scan_global_frame = LaunchConfiguration("scan_global_frame")
    scan_robot_frame = LaunchConfiguration("scan_robot_frame")
    scan_tf_pose_rate = LaunchConfiguration("scan_tf_pose_rate")
    scan_raw_path_topic = LaunchConfiguration("scan_raw_path_topic")
    scan_initial_path_topic = LaunchConfiguration("scan_initial_path_topic")
    enable_robot_control = LaunchConfiguration("enable_robot_control")
    robot_model = LaunchConfiguration("robot_model")
    robot_control_config = LaunchConfiguration("robot_control_config")
    robot_cmd_vel_topic = LaunchConfiguration("robot_cmd_vel_topic")
    robot_nav_stop_topic = LaunchConfiguration("robot_nav_stop_topic")
    robot_nav_start_topic = LaunchConfiguration("robot_nav_start_topic")
    robot_status_topic = LaunchConfiguration("robot_status_topic")
    robot_max_linear_x = LaunchConfiguration("robot_max_linear_x")
    robot_max_linear_y = LaunchConfiguration("robot_max_linear_y")
    robot_max_angular_z = LaunchConfiguration("robot_max_angular_z")
    go2_ip = LaunchConfiguration("go2_ip")
    go2_serial_number = LaunchConfiguration("go2_serial_number")
    go2_aes_128_key = LaunchConfiguration("go2_aes_128_key")
    go2_connection_method = LaunchConfiguration("go2_connection_method")
    go2_ensure_motion_mode = LaunchConfiguration("go2_ensure_motion_mode")
    go2_motion_mode = LaunchConfiguration("go2_motion_mode")
    go2_stand_on_connect = LaunchConfiguration("go2_stand_on_connect")
    go2_stand_command = LaunchConfiguration("go2_stand_command")
    go2_stand_wait_sec = LaunchConfiguration("go2_stand_wait_sec")
    go2_continuous_gait_on_connect = LaunchConfiguration("go2_continuous_gait_on_connect")
    go2_continuous_gait_value = LaunchConfiguration("go2_continuous_gait_value")
    go2_speed_level_on_connect = LaunchConfiguration("go2_speed_level_on_connect")
    go2_speed_level = LaunchConfiguration("go2_speed_level")
    go2_move_command_mode = LaunchConfiguration("go2_move_command_mode")
    go2_mcf_linear_scale = LaunchConfiguration("go2_mcf_linear_scale")
    go2_mcf_angular_scale = LaunchConfiguration("go2_mcf_angular_scale")
    go2_mcf_max_x = LaunchConfiguration("go2_mcf_max_x")
    go2_mcf_max_y = LaunchConfiguration("go2_mcf_max_y")
    go2_mcf_max_z = LaunchConfiguration("go2_mcf_max_z")
    go2_wireless_linear_scale = LaunchConfiguration("go2_wireless_linear_scale")
    go2_wireless_angular_scale = LaunchConfiguration("go2_wireless_angular_scale")
    go2_use_remote_command_from_api = LaunchConfiguration("go2_use_remote_command_from_api")
    go2_enable_builtin_obstacle_avoidance = LaunchConfiguration(
        "go2_enable_builtin_obstacle_avoidance"
    )
    b2_cmd_vel_topic = LaunchConfiguration("b2_cmd_vel_topic")
    enable_static_base_tf = LaunchConfiguration("enable_static_base_tf")
    static_base_x = LaunchConfiguration("static_base_x")
    static_base_y = LaunchConfiguration("static_base_y")
    static_base_z = LaunchConfiguration("static_base_z")
    static_base_yaw = LaunchConfiguration("static_base_yaw")
    lidar_ip = LaunchConfiguration("lidar_ip")
    host_ip = LaunchConfiguration("host_ip")
    livox_config_path = LaunchConfiguration("livox_config_path")

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("nav_bringup"),
                "launch",
                "localization.launch.py",
            ])
        ),
        launch_arguments={
            "scene_dir": scene_dir,
            "nav_lio_map_dir": nav_lio_map_dir,
            "map_name": map_name,
            "ground_pcd": ground_pcd,
            "rviz": "false",
            "launch_livox": LaunchConfiguration("launch_livox"),
            "launch_relocation": launch_relocation,
            "lidar_ip": lidar_ip,
            "host_ip": host_ip,
            "livox_config_path": livox_config_path,
        }.items(),
    )

    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("nav_bringup"),
                "launch",
                "planning.launch.py",
            ])
        ),
        launch_arguments={
            "map_pcd": map_pcd,
            "ground_pcd": ground_pcd,
            "planground_pcd": planground_pcd,
            "roi_file_path": roi_file_path,
            "waypoints_file": waypoints_file,
            "enable_pcl_publisher": enable_pcl_publisher,
            "enable_polygon_loader": enable_polygon_loader,
            "enable_dwa_client": enable_dwa_client,
            "enable_waypoint_navigator": enable_waypoint_navigator,
            "waypoint_navigator_start_topic": waypoint_navigator_start_topic,
            "enable_waypoint_monitor": enable_waypoint_monitor,
            "enable_dynamic_avoidance": enable_dynamic_avoidance,
            "enable_path_follower": enable_path_follower,
            "enable_obstacle_simulator": enable_obstacle_simulator,
            "enable_scan_planner": enable_scan_planner,
            "enable_scan_controller": enable_scan_controller,
            "enable_scan_path_adapter": enable_scan_path_adapter,
            "enable_scan_tf_pose": enable_scan_tf_pose,
            "scan_planner_config": scan_planner_config,
            "scan_body_pose_topic": scan_body_pose_topic,
            "scan_sensor_pose_topic": scan_sensor_pose_topic,
            "scan_cloud_topic": scan_cloud_topic,
            "scan_global_frame": scan_global_frame,
            "scan_robot_frame": scan_robot_frame,
            "scan_tf_pose_rate": scan_tf_pose_rate,
            "scan_raw_path_topic": scan_raw_path_topic,
            "scan_initial_path_topic": scan_initial_path_topic,
            "enable_static_base_tf": enable_static_base_tf,
            "static_base_x": static_base_x,
            "static_base_y": static_base_y,
            "static_base_z": static_base_z,
            "static_base_yaw": static_base_yaw,
            "rviz": "false",
        }.items(),
    )

    robot_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("nav_robot_control"),
                "launch",
                "robot_control.launch.py",
            ])
        ),
        launch_arguments={
            "enable_robot_control": enable_robot_control,
            "robot_model": robot_model,
            "robot_control_config": robot_control_config,
            "robot_cmd_vel_topic": robot_cmd_vel_topic,
            "robot_nav_stop_topic": robot_nav_stop_topic,
            "robot_nav_start_topic": robot_nav_start_topic,
            "robot_status_topic": robot_status_topic,
            "robot_max_linear_x": robot_max_linear_x,
            "robot_max_linear_y": robot_max_linear_y,
            "robot_max_angular_z": robot_max_angular_z,
            "go2_ip": go2_ip,
            "go2_serial_number": go2_serial_number,
            "go2_aes_128_key": go2_aes_128_key,
            "go2_connection_method": go2_connection_method,
            "go2_ensure_motion_mode": go2_ensure_motion_mode,
            "go2_motion_mode": go2_motion_mode,
            "go2_stand_on_connect": go2_stand_on_connect,
            "go2_stand_command": go2_stand_command,
            "go2_stand_wait_sec": go2_stand_wait_sec,
            "go2_continuous_gait_on_connect": go2_continuous_gait_on_connect,
            "go2_continuous_gait_value": go2_continuous_gait_value,
            "go2_speed_level_on_connect": go2_speed_level_on_connect,
            "go2_speed_level": go2_speed_level,
            "go2_move_command_mode": go2_move_command_mode,
            "go2_mcf_linear_scale": go2_mcf_linear_scale,
            "go2_mcf_angular_scale": go2_mcf_angular_scale,
            "go2_mcf_max_x": go2_mcf_max_x,
            "go2_mcf_max_y": go2_mcf_max_y,
            "go2_mcf_max_z": go2_mcf_max_z,
            "go2_wireless_linear_scale": go2_wireless_linear_scale,
            "go2_wireless_angular_scale": go2_wireless_angular_scale,
            "go2_use_remote_command_from_api": go2_use_remote_command_from_api,
            "go2_enable_builtin_obstacle_avoidance": go2_enable_builtin_obstacle_avoidance,
            "b2_cmd_vel_topic": b2_cmd_vel_topic,
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="navigation_debug_rviz",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution([
                FindPackageShare("nav_bringup"),
                "rviz",
                "navigation_debug.rviz",
            ]),
        ],
        condition=IfCondition(navigation_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("scene_dir", default_value=""),
        DeclareLaunchArgument("nav_lio_map_dir", default_value="map"),
        DeclareLaunchArgument("map_name", default_value="map.pcd"),
        DeclareLaunchArgument("map_pcd", default_value=""),
        DeclareLaunchArgument("ground_pcd", default_value=""),
        DeclareLaunchArgument("planground_pcd", default_value=""),
        DeclareLaunchArgument("roi_file_path", default_value=""),
        DeclareLaunchArgument(
            "waypoints_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("nav_planner"),
                "data",
                "waypoints.json",
            ]),
        ),
        DeclareLaunchArgument("launch_livox", default_value="true"),
        DeclareLaunchArgument("launch_relocation", default_value="true"),
        DeclareLaunchArgument("enable_pcl_publisher", default_value="true"),
        DeclareLaunchArgument("enable_polygon_loader", default_value="false"),
        DeclareLaunchArgument("enable_dwa_client", default_value="false"),
        DeclareLaunchArgument("enable_waypoint_navigator", default_value="false"),
        DeclareLaunchArgument("waypoint_navigator_start_topic", default_value="/nav_task_start"),
        DeclareLaunchArgument("enable_waypoint_monitor", default_value="false"),
        DeclareLaunchArgument("enable_dynamic_avoidance", default_value="true"),
        DeclareLaunchArgument("enable_path_follower", default_value="false"),
        DeclareLaunchArgument("enable_obstacle_simulator", default_value="false"),
        DeclareLaunchArgument("enable_scan_planner", default_value="false"),
        DeclareLaunchArgument("enable_scan_controller", default_value="false"),
        DeclareLaunchArgument("enable_scan_path_adapter", default_value="true"),
        DeclareLaunchArgument("enable_scan_tf_pose", default_value="true"),
        DeclareLaunchArgument(
            "scan_planner_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("nav_bringup"),
                "config",
                "scan_planner.yaml",
            ]),
        ),
        DeclareLaunchArgument("scan_body_pose_topic", default_value="/scan/body_pose"),
        DeclareLaunchArgument("scan_sensor_pose_topic", default_value="/lio/odom"),
        DeclareLaunchArgument("scan_cloud_topic", default_value="/lio/cloud_world"),
        DeclareLaunchArgument("scan_global_frame", default_value="map"),
        DeclareLaunchArgument("scan_robot_frame", default_value="base_footprint"),
        DeclareLaunchArgument("scan_tf_pose_rate", default_value="30.0"),
        DeclareLaunchArgument("scan_raw_path_topic", default_value="/global_path"),
        DeclareLaunchArgument("scan_initial_path_topic", default_value="/scan/initial_path"),
        DeclareLaunchArgument("enable_robot_control", default_value="false"),
        DeclareLaunchArgument("robot_model", default_value="none"),
        DeclareLaunchArgument(
            "robot_control_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("nav_robot_control"),
                "config",
                "robot_control.yaml",
            ]),
        ),
        DeclareLaunchArgument("robot_cmd_vel_topic", default_value="/cmd_vel_safe"),
        DeclareLaunchArgument("robot_nav_stop_topic", default_value="/nav_stop"),
        DeclareLaunchArgument("robot_nav_start_topic", default_value="/nav_start"),
        DeclareLaunchArgument("robot_status_topic", default_value="/robot_control/status"),
        DeclareLaunchArgument("robot_max_linear_x", default_value="0.25"),
        DeclareLaunchArgument("robot_max_linear_y", default_value="0.10"),
        DeclareLaunchArgument("robot_max_angular_z", default_value="0.40"),
        DeclareLaunchArgument("go2_ip", default_value="192.168.8.181"),
        DeclareLaunchArgument("go2_serial_number", default_value=""),
        DeclareLaunchArgument("go2_aes_128_key", default_value=""),
        DeclareLaunchArgument("go2_connection_method", default_value="LocalSTA"),
        DeclareLaunchArgument("go2_ensure_motion_mode", default_value="false"),
        DeclareLaunchArgument("go2_motion_mode", default_value="normal"),
        DeclareLaunchArgument("go2_stand_on_connect", default_value="true"),
        DeclareLaunchArgument("go2_stand_command", default_value="BalanceStand"),
        DeclareLaunchArgument("go2_stand_wait_sec", default_value="1.0"),
        DeclareLaunchArgument("go2_continuous_gait_on_connect", default_value="false"),
        DeclareLaunchArgument("go2_continuous_gait_value", default_value="1"),
        DeclareLaunchArgument("go2_speed_level_on_connect", default_value="false"),
        DeclareLaunchArgument("go2_speed_level", default_value="1"),
        DeclareLaunchArgument("go2_move_command_mode", default_value="no_reply"),
        DeclareLaunchArgument("go2_mcf_linear_scale", default_value="1.0"),
        DeclareLaunchArgument("go2_mcf_angular_scale", default_value="1.0"),
        DeclareLaunchArgument("go2_mcf_max_x", default_value="1.0"),
        DeclareLaunchArgument("go2_mcf_max_y", default_value="1.0"),
        DeclareLaunchArgument("go2_mcf_max_z", default_value="1.0"),
        DeclareLaunchArgument("go2_wireless_linear_scale", default_value="3.0"),
        DeclareLaunchArgument("go2_wireless_angular_scale", default_value="2.0"),
        DeclareLaunchArgument("go2_use_remote_command_from_api", default_value="false"),
        DeclareLaunchArgument("go2_enable_builtin_obstacle_avoidance", default_value="false"),
        DeclareLaunchArgument("b2_cmd_vel_topic", default_value="/unitree/b2/cmd_vel"),
        DeclareLaunchArgument("enable_static_base_tf", default_value="false"),
        DeclareLaunchArgument("static_base_x", default_value="0.0"),
        DeclareLaunchArgument("static_base_y", default_value="0.0"),
        DeclareLaunchArgument("static_base_z", default_value="0.0"),
        DeclareLaunchArgument("static_base_yaw", default_value="0.0"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("navigation_rviz", default_value=rviz),
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
        localization_launch,
        planning_launch,
        robot_control_launch,
        rviz_node,
    ])
