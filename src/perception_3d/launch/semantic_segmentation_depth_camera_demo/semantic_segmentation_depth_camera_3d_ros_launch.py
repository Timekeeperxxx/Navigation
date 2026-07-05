import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import TimerAction

def generate_launch_description():

  package_dir = get_package_share_directory('perception_3d')
  map_dir = os.path.join(package_dir, 'map')

  map2baselink = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen" ,
        arguments=["1.0", "0.0", "0.0", "0.0", "0.0", "0.0", "map", "base_link"]
      )

  b2c = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen" ,
        arguments=["0.4", "0.0", "0.4", "0.0", "0.0", "-0.03", "base_link", "camera_link"]
      )

  cam2optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen" ,
        arguments=["0.0", "0.0", "0.0", "-1.571", "-0.000", "-1.571", "camera_link", "mid_depth_optical_frame"]
    )

  #--------provide map/ground to static layer
  pcl_publisher = Node(
          package="mcl_3dl",
          executable="pcl_publisher",
          output="screen",
          parameters=[
              {"global_frame": 'map'},
              {"map_rotate_around_x": 1.570796327},
              {"ground_rotate_around_x": 1.570796327},
              {"map_down_sample": 0.5},
              {"ground_down_sample": 0.5},
              {"map_dir": map_dir + '/map.pcd'},
              {"ground_dir": map_dir + '/ground.pcd'},
          ]
  )  
  
  #--------semantic segmentation pc
  trt_node = Node(
      package='dddmr_semantic_segmentation',
      executable='ddrnet_ros_img_sub.py',
      parameters=[{
          'publish_colored_mask_result': True, #set to False to save cpu up tp 20% and reach more than 15 fps
          'use_class_based_mask_result': True #False use rgb based
      }],
      remappings=[
          ('/camera/camera/color/image_raw', '/realsense/mid/color/image_raw')
      ]
  )


  mask2pointcloud_node = Node(
      package='dddmr_semantic_segmentation',
      executable='semantic_segmentation2point_cloud',
      parameters=[{
          'max_distance': 4.0,
          'sample_step': 2,
          'voxel_size': 0.05,
          'exclude_class': [0]
      }],
      remappings=[
          ('/camera_info', '/realsense/mid/depth/camera_info'),
          ('/ddrnet_inferenced_mask', '/ddrnet_inferenced_mask'),
          ('/image_rect_raw', '/realsense/mid/depth/image_rect_raw')
      ]
  )

  perception_3d_yaml = os.path.join(
      get_package_share_directory('perception_3d'),
      'config', 'segmentation_depth_camera_demo',
      'semantic_segmentation_depth_camera_3d_ros.yaml'
      )

  perception_3d_ros = Node(
          package="perception_3d",
          executable="perception_3d_ros_node",
          output="screen",
          parameters = [perception_3d_yaml]
  )  

  rviz = Node(
          package="rviz2",
          executable="rviz2",
          output="screen",
          arguments=['-d', os.path.join(get_package_share_directory('perception_3d'), 'rviz', 'semantic_segmentation_depth_camera_3d.rviz')]
  )  
  
  bag_player = ExecuteProcess(
      cmd=[
          "ros2",
          "bag",
          "play",
          "--loop",
          "/root/dddmr_bags/rs435_rgbd_848x380",
      ],
      output="screen",
  )

  ld = LaunchDescription()

  ld.add_action(map2baselink)

  ld.add_action(b2c)
  ld.add_action(cam2optical)


  ld.add_action(pcl_publisher)
  ld.add_action(trt_node)
  ld.add_action(mask2pointcloud_node)
  ld.add_action(perception_3d_ros)
  ld.add_action(rviz)
  
  ld.add_action(TimerAction(period=8.0, actions=[bag_player])) #wait for engine to be loaded

  return ld