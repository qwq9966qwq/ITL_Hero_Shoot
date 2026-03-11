cmake_minimum_required(VERSION 3.11)
project(relocalization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenMP REQUIRED)

# Fetch small_gicp library
include(FetchContent)
fetchcontent_declare(
  small_gicp
  GIT_REPOSITORY https://github.com/koide3/small_gicp.git
  GIT_TAG "master"
)
fetchcontent_makeavailable(small_gicp)

add_library(${PROJECT_NAME} SHARED
  src/relocalization.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  tf2
  tf2_eigen
  tf2_ros
  pcl_conversions
)

target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX small_gicp)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "relocalization::RelocalizationNode"
  EXECUTABLE relocalization_node
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
