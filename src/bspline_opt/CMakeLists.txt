cmake_minimum_required(VERSION 3.8)
project(bspline_opt)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-O3 -Wall -g)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(bspline_opt
  src/uniform_bspline.cpp
  src/bspline_optimizer.cpp
  src/gradient_descent_optimizer.cpp
)

target_compile_features(bspline_opt PUBLIC cxx_std_14)

target_include_directories(bspline_opt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(bspline_opt
  rclcpp
  std_msgs
  visualization_msgs
  plan_env
  path_searching
)

install(TARGETS bspline_opt
  EXPORT export_bspline_opt
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_bspline_opt HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(bspline_opt)
ament_export_dependencies(
  rclcpp
  std_msgs
  visualization_msgs
  plan_env
  path_searching
)

ament_package()
