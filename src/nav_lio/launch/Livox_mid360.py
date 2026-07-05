import os
import launch.logging
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_super_lio = get_package_share_directory('nav_lio')
    config_yaml = os.path.join(pkg_super_lio, 'config', 'livox_360.yaml')
    rviz_config_file = os.path.join(pkg_super_lio, 'rviz', 'lio.rviz')

    # PGO 配置路径
    # pkg_pgo = get_package_share_directory('pgo')
    # pgo_config_path = os.path.join(pkg_pgo, 'config', 'pgo.yaml')

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RVIZ2'
    )
    rviz_flag = LaunchConfiguration('rviz')

    declare_save_map_arg = DeclareLaunchArgument(
        'save_map',
        default_value='true',
        description='Whether to save map'
    )
    declare_save_map_dir_arg = DeclareLaunchArgument(
        'save_map_dir',
        default_value='map',
        description='Directory to save map'
    )
    declare_map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='map.pcd',
        description='Map file name'
    )
    declare_lio_voxel_filter_size_arg = DeclareLaunchArgument(
        'lio_voxel_filter_size',
        default_value='0.5',
        description='前端匹配点云体素大小'
    )
    declare_lidar_time_offset_arg = DeclareLaunchArgument(
        'lidar_time_offset',
        default_value='0.0',
        description='雷达时间相对 IMU 的偏移，单位秒'
    )
    declare_estimate_gravity_arg = DeclareLaunchArgument(
        'estimate_gravity',
        default_value='true',
        description='是否让 ESKF 在运行中估计 gravity 方向'
    )
    declare_use_query_time_undistort_arg = DeclareLaunchArgument(
        'use_query_time_undistort',
        default_value='false',
        description='去畸变平移插值是否使用点自身时间'
    )
    declare_lidar_imu_roll_deg_arg = DeclareLaunchArgument(
        'lidar_imu_roll_deg',
        default_value='0.0',
        description='LiDAR 到 IMU 外参 roll 微调，单位度'
    )
    declare_lidar_imu_pitch_deg_arg = DeclareLaunchArgument(
        'lidar_imu_pitch_deg',
        default_value='0.0',
        description='LiDAR 到 IMU 外参 pitch 微调，单位度'
    )
    declare_lidar_imu_yaw_deg_arg = DeclareLaunchArgument(
        'lidar_imu_yaw_deg',
        default_value='0.0',
        description='LiDAR 到 IMU 外参 yaw 微调，单位度'
    )
    declare_loop_closure_arg = DeclareLaunchArgument(
        'loop_closure',
        default_value='false',
        description='是否在保存时生成闭环校正地图'
    )
    declare_loop_search_radius_arg = DeclareLaunchArgument(
        'loop_search_radius',
        default_value='5.0',
        description='闭环候选搜索半径，单位米'
    )
    declare_loop_icp_score_threshold_arg = DeclareLaunchArgument(
        'loop_icp_score_threshold',
        default_value='1.0',
        description='闭环 ICP fitness score 最大允许值'
    )
    declare_loop_map_ds_size_arg = DeclareLaunchArgument(
        'loop_map_ds_size',
        default_value='0.1',
        description='闭环输出地图体素大小'
    )

    super_lio_node = Node(
        package='nav_lio',
        executable='super_lio_node',
        name='super_lio_node',
        output='screen',
        parameters=[config_yaml, 
                    {"lio.map.save_map": LaunchConfiguration('save_map'),
                     "lio.map.save_map_dir": LaunchConfiguration('save_map_dir'),
                     "lio.map.map_name": LaunchConfiguration('map_name'),
                     "lio.sensor.voxel_fliter_size": LaunchConfiguration('lio_voxel_filter_size'),
                     "lio.sensor.lidar_time_offset": LaunchConfiguration('lidar_time_offset'),
                     "lio.sensor.use_query_time_undistort": LaunchConfiguration('use_query_time_undistort'),
                     "lio.kf.estimate_gravity": LaunchConfiguration('estimate_gravity'),
                     "lio.extrinsic.lidar_imu_roll_deg": LaunchConfiguration('lidar_imu_roll_deg'),
                     "lio.extrinsic.lidar_imu_pitch_deg": LaunchConfiguration('lidar_imu_pitch_deg'),
                     "lio.extrinsic.lidar_imu_yaw_deg": LaunchConfiguration('lidar_imu_yaw_deg'),
                     "lio.loop.enable": LaunchConfiguration('loop_closure'),
                     "lio.loop.search_radius": LaunchConfiguration('loop_search_radius'),
                     "lio.loop.icp_score_threshold": LaunchConfiguration('loop_icp_score_threshold'),
                     "lio.loop.map_ds_size": LaunchConfiguration('loop_map_ds_size')}],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # PGO 回环检测节点
    # pgo_node = Node(
    #     package='pgo',
    #     executable='pgo_node',
    #     name='pgo_node',
    #     output='screen',
    #     parameters=[{"config_path": pgo_config_path}],
    #     arguments=['--ros-args', '--log-level', 'info']
    # )

    # 静态变换: base_footprint -> base_link (单位矩阵, 无平移无旋转)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_footprint_to_base_link',
        arguments=['0', '0', '-0.90', '0', '-0.34', '0', 'base_link', 'base_footprint'],
        output='screen'
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='super_lio',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'warn'],
        condition=IfCondition(rviz_flag)
    )

    ld = LaunchDescription()
    ld.add_action(declare_rviz_arg)
    ld.add_action(declare_save_map_arg)
    ld.add_action(declare_save_map_dir_arg)
    ld.add_action(declare_map_name_arg)
    ld.add_action(declare_lio_voxel_filter_size_arg)
    ld.add_action(declare_lidar_time_offset_arg)
    ld.add_action(declare_estimate_gravity_arg)
    ld.add_action(declare_use_query_time_undistort_arg)
    ld.add_action(declare_lidar_imu_roll_deg_arg)
    ld.add_action(declare_lidar_imu_pitch_deg_arg)
    ld.add_action(declare_lidar_imu_yaw_deg_arg)
    ld.add_action(declare_loop_closure_arg)
    ld.add_action(declare_loop_search_radius_arg)
    ld.add_action(declare_loop_icp_score_threshold_arg)
    ld.add_action(declare_loop_map_ds_size_arg)
    ld.add_action(static_tf_node)
    ld.add_action(super_lio_node)
    # ld.add_action(pgo_node)
    ld.add_action(rviz2_node)

    return ld
