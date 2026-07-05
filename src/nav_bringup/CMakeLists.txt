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
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
