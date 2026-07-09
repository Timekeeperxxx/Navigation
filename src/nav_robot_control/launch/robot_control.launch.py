from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _model_condition(enable_robot_control, robot_model, allowed):
    checks = [f"'{name}'" for name in allowed]
    return IfCondition(
        PythonExpression(
            [
                "'",
                enable_robot_control,
                "' == 'true' and '",
                robot_model,
                "' in [",
                ", ".join(checks),
                "]",
            ]
        )
    )


def generate_launch_description():
    enable_robot_control = LaunchConfiguration("enable_robot_control")
    robot_model = LaunchConfiguration("robot_model")
    config_file = LaunchConfiguration("robot_control_config")
    cmd_vel_topic = LaunchConfiguration("robot_cmd_vel_topic")
    nav_stop_topic = LaunchConfiguration("robot_nav_stop_topic")
    nav_start_topic = LaunchConfiguration("robot_nav_start_topic")
    status_topic = LaunchConfiguration("robot_status_topic")
    max_linear_x = LaunchConfiguration("robot_max_linear_x")
    max_linear_y = LaunchConfiguration("robot_max_linear_y")
    max_angular_z = LaunchConfiguration("robot_max_angular_z")
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

    go2_bridge = Node(
        package="nav_robot_control",
        executable="go2_webrtc_bridge",
        name="go2_webrtc_bridge",
        output="screen",
        parameters=[
            config_file,
            {
                "cmd_vel_topic": cmd_vel_topic,
                "nav_stop_topic": nav_stop_topic,
                "nav_start_topic": nav_start_topic,
                "status_topic": status_topic,
                "max_linear_x": max_linear_x,
                "max_linear_y": max_linear_y,
                "max_angular_z": max_angular_z,
                "robot_ip": go2_ip,
                "serial_number": go2_serial_number,
                "aes_128_key": go2_aes_128_key,
                "connection_method": go2_connection_method,
                "ensure_motion_mode": go2_ensure_motion_mode,
                "motion_mode": go2_motion_mode,
                "stand_on_connect": go2_stand_on_connect,
                "stand_command": go2_stand_command,
                "stand_wait_sec": go2_stand_wait_sec,
                "continuous_gait_on_connect": go2_continuous_gait_on_connect,
                "continuous_gait_value": go2_continuous_gait_value,
                "speed_level_on_connect": go2_speed_level_on_connect,
                "speed_level": go2_speed_level,
                "move_command_mode": go2_move_command_mode,
                "mcf_linear_scale": go2_mcf_linear_scale,
                "mcf_angular_scale": go2_mcf_angular_scale,
                "mcf_max_x": go2_mcf_max_x,
                "mcf_max_y": go2_mcf_max_y,
                "mcf_max_z": go2_mcf_max_z,
                "wireless_linear_scale": go2_wireless_linear_scale,
                "wireless_angular_scale": go2_wireless_angular_scale,
                "use_remote_command_from_api": go2_use_remote_command_from_api,
                "enable_builtin_obstacle_avoidance": go2_enable_builtin_obstacle_avoidance,
            },
        ],
        condition=_model_condition(enable_robot_control, robot_model, ["go2", "go2_webrtc"]),
    )

    b2_bridge = Node(
        package="nav_robot_control",
        executable="b2_cmd_vel_bridge",
        name="b2_cmd_vel_bridge",
        output="screen",
        parameters=[
            config_file,
            {
                "cmd_vel_topic": cmd_vel_topic,
                "nav_stop_topic": nav_stop_topic,
                "nav_start_topic": nav_start_topic,
                "status_topic": status_topic,
                "max_linear_x": max_linear_x,
                "max_linear_y": max_linear_y,
                "max_angular_z": max_angular_z,
                "b2_cmd_vel_topic": b2_cmd_vel_topic,
            },
        ],
        condition=_model_condition(enable_robot_control, robot_model, ["b2", "b2_sdk", "b2_topic"]),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_robot_control", default_value="false"),
            DeclareLaunchArgument("robot_model", default_value="none"),
            DeclareLaunchArgument(
                "robot_control_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("nav_robot_control"), "config", "robot_control.yaml"]
                ),
            ),
            DeclareLaunchArgument("robot_cmd_vel_topic", default_value="/cmd_vel_safe"),
            DeclareLaunchArgument("robot_nav_stop_topic", default_value="/nav_stop"),
            DeclareLaunchArgument("robot_nav_start_topic", default_value="/nav_start"),
            DeclareLaunchArgument("robot_status_topic", default_value="/robot_control/status"),
            DeclareLaunchArgument("robot_max_linear_x", default_value="0.15"),
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
            go2_bridge,
            b2_bridge,
        ]
    )
