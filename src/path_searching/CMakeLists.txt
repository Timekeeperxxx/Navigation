cmake_minimum_required(VERSION 3.8)
project(path_searching)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-O3 -Wall -g)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(path_searching
  src/dyn_a_star.cpp
)

target_compile_features(path_searching PUBLIC cxx_std_14)

target_include_directories(path_searching PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(path_searching
  rclcpp
  visualization_msgs
  plan_env
)

install(TARGETS path_searching
  EXPORT export_path_searching
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_path_searching HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(path_searching)
ament_export_dependencies(
  rclcpp
  visualization_msgs
  plan_env
)

ament_package()
