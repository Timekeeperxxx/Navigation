cmake_minimum_required(VERSION 3.8)
project(traj_utils)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-O3 -Wall -g)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(path_searching REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(traj_utils
  src/planning_visualization.cpp
  src/polynomial_traj.cpp
)

target_compile_features(traj_utils PUBLIC cxx_std_14)

target_include_directories(traj_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(traj_utils
  rclcpp
  std_msgs
  visualization_msgs
  bspline_opt
  path_searching
)

install(TARGETS traj_utils
  EXPORT export_traj_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_traj_utils HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(traj_utils)
ament_export_dependencies(
  rclcpp
  std_msgs
  visualization_msgs
  bspline_opt
  path_searching
)

ament_package()
