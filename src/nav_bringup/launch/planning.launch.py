from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
    enable_static_base_tf = LaunchConfiguration("enable_static_base_tf")
    global_planner_config = LaunchConfiguration("global_planner_config")
    static_base_x = LaunchConfiguration("static_base_x")
    static_base_y = LaunchConfiguration("static_base_y")
    static_base_z = LaunchConfiguration("static_base_z")
    static_base_yaw = LaunchConfiguration("static_base_yaw")
    waypoints_file = LaunchConfiguration("waypoints_file")
    enable_rviz = LaunchConfiguration("rviz")

    default_global_planner_config = PathJoinSubstitution([
        FindPackageShare("nav_bringup"),
        "config",
        "global_planner.yaml",
    ])
    dynamic_avoidance_config = PathJoinSubstitution([
        FindPackageShare("nav_bringup"),
        "config",
        "dynamic_avoidance.yaml",
    ])
    default_scan_planner_config = PathJoinSubstitution([
        FindPackageShare("nav_bringup"),
        "config",
        "scan_planner.yaml",
    ])

    pcl_publisher = Node(
        package="nav_bringup",
        executable="nav_pcd_map_publisher.py",
        name="pcl_publisher",
        output="screen",
        parameters=[
            global_planner_config,
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
                "nav_start_topic": waypoint_navigator_start_topic,
                "waypoint_reached_topic": "/waypoint_reached",
                "waypoint_context_topic": "/nav/task_waypoint_context",
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
                "nav_status_topic": "/nav_status",
                "waypoint_context_topic": "/nav/task_waypoint_context",
                "goal_yaw_topic": "goal_yaw",
                "nav_start_topic": "/nav_start",
                "nav_stop_topic": "/nav_stop",
                "reach_tolerance_xy": 0.12,
                "reach_tolerance_z": 1.0,
                "reach_tolerance_yaw": 0.10,
                "timeout_sec": 0.0,
            }
        ],
        condition=IfCondition(enable_waypoint_monitor),
    )

    dynamic_avoidance_monitor = Node(
        package="nav_planner",
        executable="dynamic_avoidance_monitor.py",
        name="dynamic_avoidance_monitor",
        output="screen",
        parameters=[dynamic_avoidance_config],
        condition=IfCondition(enable_dynamic_avoidance),
    )

    path_follower = Node(
        package="nav_planner",
        executable="nav_path_follower.py",
        name="nav_path_follower",
        output="screen",
        parameters=[dynamic_avoidance_config],
        condition=IfCondition(enable_path_follower),
    )

    obstacle_simulator = Node(
        package="nav_planner",
        executable="local_obstacle_simulator.py",
        name="local_obstacle_simulator",
        output="screen",
        parameters=[dynamic_avoidance_config],
        condition=IfCondition(enable_obstacle_simulator),
    )

    scan_path_adapter = Node(
        package="nav_bringup",
        executable="scan_initial_path_adapter.py",
        name="scan_initial_path_adapter",
        output="screen",
        parameters=[
            {
                "input_path_topic": scan_raw_path_topic,
                "output_path_topic": scan_initial_path_topic,
                "min_point_spacing": 0.5,
                "max_points": 120,
            }
        ],
        condition=IfCondition(
            PythonExpression([
                "'",
                enable_scan_planner,
                "' in ['true', 'True', '1'] and '",
                enable_scan_path_adapter,
                "' in ['true', 'True', '1']",
            ])
        ),
    )

    scan_tf_pose = Node(
        package="nav_bringup",
        executable="scan_tf_pose_publisher.py",
        name="scan_tf_pose_publisher",
        output="screen",
        parameters=[{
            "global_frame": scan_global_frame,
            "robot_frame": scan_robot_frame,
            "output_topic": scan_body_pose_topic,
            "publish_rate_hz": scan_tf_pose_rate,
        }],
        condition=IfCondition(
            PythonExpression([
                "('",
                enable_scan_planner,
                "' in ['true', 'True', '1'] or '",
                enable_scan_controller,
                "' in ['true', 'True', '1']) and '",
                enable_scan_tf_pose,
                "' in ['true', 'True', '1']",
            ])
        ),
    )

    scan_planner = Node(
        package="scan_planner",
        executable="scan_planner_node",
        name="scan_planner_node",
        output="screen",
        parameters=[
            scan_planner_config,
            {
                "body_pose_topic": scan_body_pose_topic,
            },
        ],
        remappings=[
            ("/initial_path", scan_initial_path_topic),
            ("/grid_map/body_pose", scan_body_pose_topic),
            ("/grid_map/sensor_pose", scan_sensor_pose_topic),
            ("/grid_map/cloud", scan_cloud_topic),
        ],
        condition=IfCondition(enable_scan_planner),
    )

    scan_controller = Node(
        package="scan_planner",
        executable="closed_loop_controller",
        name="closed_loop_controller",
        output="screen",
        parameters=[
            scan_planner_config,
            {
                "body_pose_topic": scan_body_pose_topic,
            },
        ],
        condition=IfCondition(enable_scan_controller),
    )

    static_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_base_footprint",
        arguments=[
            static_base_x,
            static_base_y,
            static_base_z,
            static_base_yaw,
            "0.0",
            "0.0",
            "map",
            "base_footprint",
        ],
        condition=IfCondition(enable_static_base_tf),
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
            default_value=default_scan_planner_config,
        ),
        DeclareLaunchArgument("scan_body_pose_topic", default_value="/scan/body_pose"),
        DeclareLaunchArgument("scan_sensor_pose_topic", default_value="/lio/odom"),
        DeclareLaunchArgument("scan_cloud_topic", default_value="/lio/cloud_world"),
        DeclareLaunchArgument("scan_global_frame", default_value="map"),
        DeclareLaunchArgument("scan_robot_frame", default_value="base_footprint"),
        DeclareLaunchArgument("scan_tf_pose_rate", default_value="30.0"),
        DeclareLaunchArgument("scan_raw_path_topic", default_value="/global_path"),
        DeclareLaunchArgument("scan_initial_path_topic", default_value="/scan/initial_path"),
        DeclareLaunchArgument("enable_static_base_tf", default_value="false"),
        DeclareLaunchArgument(
            "global_planner_config",
            default_value=default_global_planner_config,
        ),
        DeclareLaunchArgument("static_base_x", default_value="0.0"),
        DeclareLaunchArgument("static_base_y", default_value="0.0"),
        DeclareLaunchArgument("static_base_z", default_value="0.0"),
        DeclareLaunchArgument("static_base_yaw", default_value="0.0"),
        DeclareLaunchArgument("rviz", default_value="false"),
        pcl_publisher,
        global_planner,
        polygon_loader,
        dwa_client,
        waypoint_navigator,
        waypoint_monitor,
        dynamic_avoidance_monitor,
        path_follower,
        obstacle_simulator,
        scan_path_adapter,
        scan_tf_pose,
        scan_planner,
        scan_controller,
        static_base_tf,
        rviz_node,
    ])
