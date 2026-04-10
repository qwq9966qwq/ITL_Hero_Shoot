cmake_minimum_required(VERSION 3.8)
project(lob_shot_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(projectile_motion REQUIRED)
find_package(lob_shot_aiming REQUIRED)
find_package(utils REQUIRED)

add_executable(lob_shot_manager_node
  src/lob_shot_manager_node.cpp
  src/main.cpp
)

target_include_directories(lob_shot_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(lob_shot_manager_node PUBLIC cxx_std_17)

ament_target_dependencies(lob_shot_manager_node PUBLIC
  rclcpp
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  geometry_msgs
  visualization_msgs
  tf2_geometry_msgs
  projectile_motion
  lob_shot_aiming
)

target_link_libraries(lob_shot_manager_node PUBLIC utils::utils)

install(
  TARGETS lob_shot_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
