from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value):
    return str(value).lower() in ("1", "true", "yes", "on")


def _as_float(value):
    return float(str(value))


def _as_int(value):
    return int(str(value))


def _setup(context, *args, **kwargs):
    is_real_world = _as_bool(LaunchConfiguration("is_real_world").perform(context))
    navi_mode = _as_int(LaunchConfiguration("navi_mode").perform(context))
    sensor_type = LaunchConfiguration("sensor_type").perform(context)
    controller_mode = LaunchConfiguration("controller_mode").perform(context)
    use_robot_state_publisher = _as_bool(LaunchConfiguration("use_robot_state_publisher").perform(context))
    use_odom_visualization = _as_bool(LaunchConfiguration("use_odom_visualization").perform(context))
    use_sim_map = _as_bool(LaunchConfiguration("use_sim_map").perform(context))
    use_local_sensing = _as_bool(LaunchConfiguration("use_local_sensing").perform(context))
    use_gpu = _as_bool(LaunchConfiguration("use_gpu").perform(context))
    use_pcd_map = _as_bool(LaunchConfiguration("use_pcd_map").perform(context))
    pcd_map_file = LaunchConfiguration("pcd_map_file").perform(context)
    robot_description_file = LaunchConfiguration("robot_description_file").perform(context)

    init_x = _as_float(LaunchConfiguration("init_x").perform(context))
    init_y = _as_float(LaunchConfiguration("init_y").perform(context))
    init_z = _as_float(LaunchConfiguration("init_z").perform(context))

    max_vel = _as_float(LaunchConfiguration("max_vel").perform(context))
    max_acc = _as_float(LaunchConfiguration("max_acc").perform(context))
    planning_horizon = _as_float(LaunchConfiguration("planning_horizon").perform(context))
    map_size_x = _as_float(LaunchConfiguration("map_size_x").perform(context))
    map_size_y = _as_float(LaunchConfiguration("map_size_y").perform(context))
    map_size_z = _as_float(LaunchConfiguration("map_size_z").perform(context))

    body_pose_topic = "/LIO/odom_vehicle" if is_real_world else "/quad_0/body_pose"
    sensor_pose_topic = (
        "/LIO/odom_imu"
        if is_real_world
        else ("/quad_0/camera_pose" if sensor_type == "depth" else "/quad_0/lidar_pose")
    )
    cloud_topic = "/LIO/clouds_lidar" if is_real_world else "/pcl_render_node/cloud"
    cloud_is_world = False if is_real_world else True
    depth_topic = "/camera/aligned_depth_to_color/image_raw" if is_real_world else "/pcl_render_node/depth"
    cx = 317.19183349609375 if is_real_world else 321.04638671875
    cy = 256.4806823730469 if is_real_world else 243.44969177246094
    fx = 609.5884399414062 if is_real_world else 387.229248046875
    fy = 609.22021484375 if is_real_world else 387.229248046875

    scan_planner_params = {
        "body_pose_topic": body_pose_topic,
        "grid_map/sensor_type": sensor_type,
        "grid_map/cloud_is_world": cloud_is_world,
        "grid_map/need_extrinsic": is_real_world,
        "fsm/navi_mode": navi_mode,
        "fsm/thresh_replan": 1.0,
        "fsm/thresh_no_replan": 0.1,
        "fsm/planning_horizon": planning_horizon,
        "fsm/emergency_time_": 1.0,
        "fsm/fail_safe": True,
        "fsm/max_replan_fail_count": 1000,
        "fsm/is_real_world": is_real_world,
        "grid_map/resolution": 0.05,
        "grid_map/sliding_map_size_x": 10.0,
        "grid_map/sliding_map_size_y": 10.0,
        "grid_map/sliding_map_size_z": 5.0,
        "grid_map/map_sliding_thresh": 0.2,
        "grid_map/double_cylinder_radius": 0.25,
        "grid_map/double_cylinder_offset": 0.18,
        "grid_map/obstacles_inflation_z_up": 0.1,
        "grid_map/obstacles_inflation_z_down": 0.4,
        "grid_map/cx": cx,
        "grid_map/cy": cy,
        "grid_map/fx": fx,
        "grid_map/fy": fy,
        "grid_map/depth_filter_maxdist": 3.0,
        "grid_map/depth_filter_mindist": 0.3,
        "grid_map/depth_filter_margin": 1,
        "grid_map/k_depth_scaling_factor": 1000.0,
        "grid_map/skip_pixel": 2,
        "grid_map/p_hit": 0.85,
        "grid_map/p_miss": 0.30,
        "grid_map/p_min": 0.12,
        "grid_map/p_max": 0.98,
        "grid_map/p_occ": 0.80,
        "grid_map/max_ray_length": 5.0,
        "grid_map/vis_height": 0.3,
        "manager/max_vel": max_vel,
        "manager/max_acc": max_acc,
        "manager/max_jerk": 4.0,
        "manager/control_points_distance": 0.2,
        "manager/feasibility_tolerance": 0.5,
        "manager/planning_horizon": planning_horizon,
        "optimization/lambda_smooth": 1.0,
        "optimization/lambda_collision": 1.0,
        "optimization/lambda_feasibility": 0.1,
        "optimization/lambda_fitness": 1.0,
        "optimization/dist0": 0.2,
        "optimization/max_vel": max_vel,
        "optimization/vel_tolerance": 1.0,
        "optimization/max_acc": max_acc,
        "optimization/acc_tolerance": 1.0,
    }

    actions = [
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=[scan_planner_params],
            remappings=[
                ("/grid_map/body_pose", body_pose_topic),
                ("/grid_map/sensor_pose", sensor_pose_topic),
                ("/grid_map/cloud", cloud_topic),
                ("/grid_map/depth", depth_topic),
            ],
        )
    ]

    if use_sim_map and not is_real_world:
        if use_pcd_map:
            actions.append(
                Node(
                    package="map_generator",
                    executable="map_pub",
                    name="map_pub",
                    output="screen",
                    arguments=[pcd_map_file],
                    parameters=[
                        {
                            "frame_id": "world",
                            "publish_rate": 0.2,
                            "cloud_topic": "/map_generator/global_cloud",
                            "downsample_res": 0.1,
                        }
                    ],
                )
            )
        else:
            actions.append(
                Node(
                    package="mockamap",
                    executable="mockamap_node",
                    name="mockamap_node",
                    output="screen",
                    parameters=[
                        {
                            "seed": 127,
                            "update_freq": 0.5,
                            "resolution": 0.1,
                            "x_length": int(map_size_x),
                            "y_length": int(map_size_y),
                            "z_length": int(map_size_z),
                            "type": 2,
                            "complexity": 0.05,
                            "fill": 0.12,
                            "fractal": 1,
                            "attenuation": 0.1,
                            "width_min": 0.2,
                            "width_max": 0.8,
                            "height_min": 2.0,
                            "height_max": 2.0,
                            "obstacle_number": 500,
                            "surface_resolution": 0.05,
                        }
                    ],
                            remappings=[("mock_map", "/map_generator/global_cloud")],
                )
            )

    if use_local_sensing and not is_real_world:
        local_sensing_params = {
            "sensor_type": sensor_type,
            "body_pose_topic": body_pose_topic,
            "map/x_size": map_size_x,
            "map/y_size": map_size_y,
            "map/z_size": map_size_z,
        }
        if use_gpu:
            local_sensing_params["use_global_map_topic"] = not use_pcd_map

        actions.append(
            Node(
                package="local_sensing_node",
                executable="opengl_render_node" if use_gpu else "pcl_render_node",
                name="pcl_render_node",
                output="screen",
                arguments=[pcd_map_file] if use_gpu else [],
                parameters=[local_sensing_params],
                remappings=[("~/global_map", "/map_generator/global_cloud")],
            )
        )

    if controller_mode == "open_loop":
        actions.append(
            Node(
                package="scan_planner",
                executable="open_loop_controller",
                name="open_loop_controller",
                output="screen",
                parameters=[
                    {
                        "body_pose_topic": body_pose_topic,
                        "init_x": init_x,
                        "init_y": init_y,
                        "init_z": init_z,
                    }
                ],
            )
        )
    elif controller_mode == "closed_loop":
        closed_loop_params = {
            "body_pose_topic": body_pose_topic,
            "time_forward": 0.8,
            "heading_error_threshold": 0.8,
            "kp_pos": 0.8,
            "kp_yaw": 1.5,
            "max_vx": max_vel,
            "max_vy": 0.35,
            "max_vyaw": 1.0,
            "finish_dist": 0.15,
        }
        actions.append(
            Node(
                package="scan_planner",
                executable="closed_loop_controller",
                name="closed_loop_controller",
                output="screen",
                parameters=[closed_loop_params],
            )
        )
        if not is_real_world:
            actions.append(
                Node(
                    package="scan_planner",
                    executable="go2_kinematic_sim",
                    name="go2_kinematic_sim",
                    output="screen",
                    parameters=[
                        {
                            "body_pose_topic": body_pose_topic,
                            "init_x": init_x,
                            "init_y": init_y,
                            "init_z": init_z,
                            "max_vx": max_vel,
                            "max_vy": 0.35,
                            "max_vyaw": 1.0,
                        }
                    ],
                )
            )
    else:
        actions.append(LogInfo(msg=f"Unknown controller_mode '{controller_mode}', no controller launched."))

    actions.append(
        Node(
            package="scan_planner",
            executable="go2_gait_publisher",
            name="go2_gait_publisher",
            output="screen",
            parameters=[{"body_pose_topic": body_pose_topic}],
        )
    )

    if use_odom_visualization:
        actions.append(
            Node(
                package="odom_visualization",
                executable="odom_visualization",
                name="odom_visualization",
                output="screen",
                parameters=[{"body_pose_topic": body_pose_topic}],
            )
        )

    if use_robot_state_publisher:
        robot_description_path = Path(robot_description_file)
        if robot_description_path.is_file():
            actions.append(
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    name="go2_robot_state_publisher",
                    output="screen",
                    parameters=[{"robot_description": robot_description_path.read_text()}],
                )
            )
        else:
            actions.append(LogInfo(msg="robot_description_file is empty or missing; robot_state_publisher not launched."))

    return actions


def generate_launch_description():
    default_robot_description = PathJoinSubstitution(
        [FindPackageShare("go2_description"), "urdf", "go2_description.urdf"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("is_real_world", default_value="false"),
            DeclareLaunchArgument("navi_mode", default_value="1"),
            DeclareLaunchArgument("sensor_type", default_value="lidar"),
            DeclareLaunchArgument("controller_mode", default_value="closed_loop"),
            DeclareLaunchArgument("init_x", default_value="-19.0"),
            DeclareLaunchArgument("init_y", default_value="1.0"),
            DeclareLaunchArgument("init_z", default_value="0.3"),
            DeclareLaunchArgument("max_vel", default_value="0.75"),
            DeclareLaunchArgument("max_acc", default_value="0.5"),
            DeclareLaunchArgument("planning_horizon", default_value="7.5"),
            DeclareLaunchArgument("map_size_x", default_value="40.0"),
            DeclareLaunchArgument("map_size_y", default_value="40.0"),
            DeclareLaunchArgument("map_size_z", default_value="5.0"),
            DeclareLaunchArgument("use_sim_map", default_value="true"),
            DeclareLaunchArgument("use_local_sensing", default_value="true"),
            DeclareLaunchArgument("use_gpu", default_value="false"),
            DeclareLaunchArgument("use_pcd_map", default_value="false"),
            DeclareLaunchArgument("pcd_map_file", default_value=""),
            DeclareLaunchArgument("use_robot_state_publisher", default_value="true"),
            DeclareLaunchArgument("use_odom_visualization", default_value="true"),
            DeclareLaunchArgument("robot_description_file", default_value=default_robot_description),
            OpaqueFunction(function=_setup),
        ]
    )
