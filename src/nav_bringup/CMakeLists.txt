cmake_minimum_required(VERSION 3.8)
project(nav_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pcd_debug_publisher.py
  scripts/nav_pcd_map_publisher.py
  scripts/rviz_2d_goal_planner.py
  scripts/scan_initial_path_adapter.py
  scripts/scan_tf_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
