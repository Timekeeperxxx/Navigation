from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_pcd = LaunchConfiguration("map_pcd")
    ground_pcd = LaunchConfiguration("ground_pcd")
    planground_pcd = LaunchConfiguration("planground_pcd")
    roi_file_path = LaunchConfiguration("roi_file_path")
    enable_pcl_publisher = LaunchConfiguration("enable_pcl_publisher")
    enable_polygon_loader = LaunchConfiguration("enable_polygon_loader")
    enable_dwa_client = LaunchConfiguration("enable_dwa_client")
    enable_waypoint_navigator = LaunchConfiguration("enable_waypoint_navigator")
    enable_waypoint_monitor = LaunchConfiguration("enable_waypoint_monitor")
    waypoints_file = LaunchConfiguration("waypoints_file")
    enable_rviz = LaunchConfiguration("rviz")

    global_planner_config = PathJoinSubstitution([
        FindPackageShare("nav_bringup"),
        "config",
        "global_planner.yaml",
    ])

    pcl_publisher = Node(
        package="nav_bringup",
        executable="nav_pcd_map_publisher.py",
        output="screen",
        parameters=[
            {
                "map_dir": map_pcd,
                "ground_dir": ground_pcd,
                "planground_dir": planground_pcd,
            },
        ],
        condition=IfCondition(enable_pcl_publisher),
    )

    global_planner = Node(
        package="nav_planner",
        executable="global_planner_node",
        output="screen",
        parameters=[
            global_planner_config,
            {"roi_file_path": roi_file_path},
        ],
    )

    polygon_loader = Node(
        package="nav_planner",
        executable="load_polygon_from_file.py",
        output="screen",
        parameters=[
            {
                "clicked_points_file": roi_file_path,
                "frame_id": "map",
                "polygon_topic": "/loaded_polygon",
                "marker_topic": "/loaded_polygon_marker",
                "points_marker_topic": "/loaded_points_markers",
                "points_stamped_topic": "/loaded_points",
            }
        ],
        condition=IfCondition(enable_polygon_loader),
    )

    dwa_client = Node(
        package="nav_planner",
        executable="clicked_dwa_planner_client.py",
        name="clicked_dwa_planner_client",
        output="screen",
        parameters=[
            {
                "clicked_point_topic": "/clicked_point",
                "goal_pose_topic": "/goal_pose",
                "action_name": "/get_dwa_plan",
                "frame_id": "map",
                "activate_threading": True,
            }
        ],
        condition=IfCondition(enable_dwa_client),
    )

    waypoint_navigator = Node(
        package="nav_planner",
        executable="waypoint_navigator_from_json.py",
        name="waypoint_navigator",
        output="screen",
        parameters=[
            {
                "waypoints_file": waypoints_file,
                "frame_id": "map",
                "clicked_point_topic": "/clicked_point",
                "nav_start_topic": "/nav_start",
                "waypoint_reached_topic": "/waypoint_reached",
            }
        ],
        condition=IfCondition(enable_waypoint_navigator),
    )

    waypoint_monitor = Node(
        package="nav_planner",
        executable="waypoint_progress_monitor.py",
        name="waypoint_progress_monitor",
        output="screen",
        parameters=[
            {
                "global_frame": "map",
                "robot_frame": "base_footprint",
                "clicked_point_topic": "/clicked_point",
                "goal_pose_topic": "/goal_pose",
                "waypoint_reached_topic": "/waypoint_reached",
                "status_topic": "/nav/waypoint_progress",
                "reach_tolerance_xy": 0.25,
                "reach_tolerance_z": 1.0,
                "timeout_sec": 0.0,
            }
        ],
        condition=IfCondition(enable_waypoint_monitor),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="navigation_planning_rviz",
        arguments=[
            "-d",
            PathJoinSubstitution([
                FindPackageShare("nav_planner"),
                "rviz",
                "path_planning_on_static_layer.rviz",
            ]),
        ],
        condition=IfCondition(enable_rviz),
    )

    return LaunchDescription([
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
        DeclareLaunchArgument("enable_pcl_publisher", default_value="true"),
        DeclareLaunchArgument("enable_polygon_loader", default_value="false"),
        DeclareLaunchArgument("enable_dwa_client", default_value="false"),
        DeclareLaunchArgument("enable_waypoint_navigator", default_value="false"),
        DeclareLaunchArgument("enable_waypoint_monitor", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        pcl_publisher,
        global_planner,
        polygon_loader,
        dwa_client,
        waypoint_navigator,
        waypoint_monitor,
        rviz_node,
    ])
