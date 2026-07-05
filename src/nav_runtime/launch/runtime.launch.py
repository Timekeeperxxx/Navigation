from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("workspace_dir", default_value=""),
            DeclareLaunchArgument("script_dir", default_value=""),
            DeclareLaunchArgument("command_topic", default_value="/nav/command_json"),
            DeclareLaunchArgument("legacy_command_topic", default_value="/nav_command_json"),
            DeclareLaunchArgument("status_topic", default_value="/nav/runtime_status"),
            DeclareLaunchArgument("result_topic", default_value="/nav/command_result"),
            Node(
                package="nav_runtime",
                executable="runtime_node",
                name="nav_runtime",
                output="screen",
                parameters=[
                    {
                        "workspace_dir": LaunchConfiguration("workspace_dir"),
                        "script_dir": LaunchConfiguration("script_dir"),
                        "command_topic": LaunchConfiguration("command_topic"),
                        "legacy_command_topic": LaunchConfiguration("legacy_command_topic"),
                        "status_topic": LaunchConfiguration("status_topic"),
                        "result_topic": LaunchConfiguration("result_topic"),
                    }
                ],
            ),
        ]
    )
