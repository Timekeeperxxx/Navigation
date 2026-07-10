from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_scene_dir = "/home/jetson/Project/BOTDOG/MAPS/Scene23_多楼层"
    default_map_pcd = f"{default_scene_dir}/map.pcd"
    default_ground_pcd = f"{default_scene_dir}/terrain_map_20260702_213550_ground.pcd"
    default_planground_pcd = (
        f"{default_scene_dir}/terrain_map_20260702_213550_base_footprint_fill.pcd"
    )

    map_pcd = LaunchConfiguration("map_pcd")
    ground_pcd = LaunchConfiguration("ground_pcd")
    planground_pcd = LaunchConfiguration("planground_pcd")
    open_rviz = LaunchConfiguration("open_rviz")

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
            "global_planner_config": PathJoinSubstitution([
                FindPackageShare("nav_bringup"),
                "config",
                "global_planner_rviz_test.yaml",
            ]),
            "enable_pcl_publisher": "true",
            "enable_dynamic_avoidance": "false",
            "enable_path_follower": "false",
            "enable_obstacle_simulator": "false",
            "enable_static_base_tf": "false",
            "rviz": "false",
        }.items(),
    )

    rviz_goal_planner = Node(
        package="nav_bringup",
        executable="rviz_2d_goal_planner.py",
        name="rviz_2d_goal_planner",
        output="screen",
        parameters=[
            {
                "input_goal_topic": "/rviz_2d_goal",
                "input_point_topic": "/rviz_ground_point",
                "output_goal_pose_topic": "/goal_pose",
                "ground_pcd": ground_pcd,
                "planground_pcd": planground_pcd,
                "global_frame": "map",
                "robot_frame": "base_footprint",
                "max_snap_xy_distance": 0.6,
                "snap_xy_to_cloud_point": True,
                "snap_prefer_source": "ground",
                "planner_ready_topic": "/nav/planner_ready",
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_planning_test",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution([
                FindPackageShare("nav_bringup"),
                "rviz",
                "rviz_planning_test.rviz",
            ]),
        ],
        condition=IfCondition(open_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("map_pcd", default_value=default_map_pcd),
        DeclareLaunchArgument("ground_pcd", default_value=default_ground_pcd),
        DeclareLaunchArgument("planground_pcd", default_value=default_planground_pcd),
        DeclareLaunchArgument("open_rviz", default_value="true"),
        planning_launch,
        rviz_goal_planner,
        rviz_node,
    ])
