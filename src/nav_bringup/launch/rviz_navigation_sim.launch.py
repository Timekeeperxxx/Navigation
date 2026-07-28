"""Offline navigation simulation for RViz.

This launch intentionally contains no hardware driver or robot command bridge.
The SCAN controller output is filtered by the dynamic-avoidance monitor and is
consumed only by b2_kinematic_sim, which publishes simulated odometry and TF.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_pcd = LaunchConfiguration("map_pcd")
    ground_pcd = LaunchConfiguration("ground_pcd")
    planground_pcd = LaunchConfiguration("planground_pcd")
    init_x = LaunchConfiguration("init_x")
    init_y = LaunchConfiguration("init_y")
    init_z = LaunchConfiguration("init_z")
    init_yaw = LaunchConfiguration("init_yaw")
    open_rviz = LaunchConfiguration("open_rviz")

    planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav_bringup"), "launch", "planning.launch.py"]
            )
        ),
        launch_arguments={
            "map_pcd": map_pcd,
            "ground_pcd": ground_pcd,
            "planground_pcd": planground_pcd,
            # Dense clouds are worthwhile in the offline RViz workflow.  The
            # normal hardware launch keeps its lighter production defaults.
            "map_down_sample": "0.10",
            "ground_down_sample": "0.10",
            # A 0.05 m cloud leaves about 91k planning nodes in Scene5 and
            # makes the global point-cloud A* needlessly expensive.  0.10 m
            # still gives two samples inside the 0.20 m expansion radius.
            "planground_down_sample": "0.10",
            "enable_pcl_publisher": "true",
            "enable_dynamic_avoidance": "true",
            # Offline simulation starts immediately after a goal is selected.
            "dynamic_require_nav_start": "false",
            "enable_path_follower": "false",
            "enable_obstacle_simulator": "true",
            "enable_scan_planner": "true",
            "enable_scan_controller": "true",
            "enable_scan_path_adapter": "true",
            "enable_scan_tf_pose": "false",
            "enable_static_base_tf": "false",
            "scan_body_pose_topic": "/sim/body_pose",
            "scan_sensor_pose_topic": "/sim/body_pose",
            "scan_cloud_topic": "/nav/local_obstacles",
            # Use the debug RViz below, not planning.launch.py's legacy config.
            "rviz": "false",
        }.items(),
    )

    b2_kinematic_sim = Node(
        package="scan_planner",
        executable="b2_kinematic_sim",
        name="b2_kinematic_sim",
        output="screen",
        parameters=[
            {
                "body_pose_topic": "/sim/body_pose",
                "init_x": ParameterValue(init_x, value_type=float),
                "init_y": ParameterValue(init_y, value_type=float),
                "init_z": ParameterValue(init_z, value_type=float),
                "init_yaw": ParameterValue(init_yaw, value_type=float),
                "max_vx": 0.25,
                "max_vy": 0.08,
                "max_vyaw": 0.30,
                "cmd_timeout": 0.30,
                "sim_rate": 100.0,
                "publish_tf": True,
                "frame_id": "map",
                "child_frame_id": "base_footprint",
                # Track base height from the static ground surface below.
                # Missing local ground leaves init_z/current z unchanged.
                "terrain_z_tracking_enabled": True,
                "execution_path_topic": "/scan/execution_path",
                # Ground is authoritative for simulated base height. Choosing
                # the vertically continuous layer handles overlapping floors
                # without feeding SCAN's output path back into its odometry.
                "terrain_ground_topic": "/mapground",
                "terrain_ground_bucket_size": 0.20,
                "terrain_ground_xy_tolerance": 0.15,
                "terrain_ground_max_layer_distance": 0.50,
                "terrain_body_height": 0.32,
                "terrain_path_timeout": 2.0,
                "terrain_max_path_slope": 0.70,
                "terrain_max_z_rate": 0.30,
                "terrain_max_projection_distance": 1.0,
            }
        ],
        remappings=[("cmd_vel", "/cmd_vel_safe")],
    )

    sim_markers = Node(
        package="nav_bringup",
        executable="sim_navigation_markers.py",
        name="sim_navigation_markers",
        output="screen",
        parameters=[
            {
                "pose_topic": "/sim/body_pose",
                "goal_topic": "/goal_pose",
                "marker_topic": "/nav/sim_visual_markers",
                "global_frame": "map",
                # Keep these identical to scan_planner.yaml:
                # radius=0.27, centre=-0.425 +/- 0.205.
                "footprint_radius": 0.27,
                "footprint_center_offsets": [-0.22, -0.63],
                "footprint_height": 0.32,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="navigation_sim_rviz",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("nav_bringup"),
                    "rviz",
                    "navigation_debug.rviz",
                ]
            ),
        ],
        condition=IfCondition(open_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_pcd"),
            DeclareLaunchArgument("ground_pcd"),
            DeclareLaunchArgument("planground_pcd"),
            DeclareLaunchArgument("init_x", default_value="0.0"),
            DeclareLaunchArgument("init_y", default_value="0.0"),
            DeclareLaunchArgument("init_z", default_value="0.0"),
            DeclareLaunchArgument("init_yaw", default_value="0.0"),
            DeclareLaunchArgument("open_rviz", default_value="true"),
            planning,
            b2_kinematic_sim,
            sim_markers,
            rviz_node,
        ]
    )
