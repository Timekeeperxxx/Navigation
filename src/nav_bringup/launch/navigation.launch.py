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
    enable_waypoint_monitor = LaunchConfiguration("enable_waypoint_monitor")
    enable_dynamic_avoidance = LaunchConfiguration("enable_dynamic_avoidance")
    enable_path_follower = LaunchConfiguration("enable_path_follower")
    enable_obstacle_simulator = LaunchConfiguration("enable_obstacle_simulator")
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
            "enable_waypoint_monitor": enable_waypoint_monitor,
            "enable_dynamic_avoidance": enable_dynamic_avoidance,
            "enable_path_follower": enable_path_follower,
            "enable_obstacle_simulator": enable_obstacle_simulator,
            "enable_static_base_tf": enable_static_base_tf,
            "static_base_x": static_base_x,
            "static_base_y": static_base_y,
            "static_base_z": static_base_z,
            "static_base_yaw": static_base_yaw,
            "rviz": "false",
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
        DeclareLaunchArgument("enable_waypoint_monitor", default_value="false"),
        DeclareLaunchArgument("enable_dynamic_avoidance", default_value="true"),
        DeclareLaunchArgument("enable_path_follower", default_value="false"),
        DeclareLaunchArgument("enable_obstacle_simulator", default_value="false"),
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
        rviz_node,
    ])
