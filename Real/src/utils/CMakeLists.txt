cmake_minimum_required(VERSION 3.8)
project(utils)

# 默认为C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)

add_library(${PROJECT_NAME} STATIC
  src/logger.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
  ${fmt_INCLUDE_DIRS}
)


if(DEFINED EIGEN3_DEFINITIONS)
  target_compile_definitions(${PROJECT_NAME} PUBLIC ${EIGEN3_DEFINITIONS})
endif()

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBRARIES}
  yaml-cpp
  fmt
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})


ament_export_include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 导出库
ament_export_libraries(
  ${PROJECT_NAME}
  yaml-cpp
  ${OpenCV_LIBS}
  ${EIGEN3_LIBRARIES}
  fmt
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  OpenCV
  Eigen3
  yaml-cpp
  fmt
)

# 导出目标
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)


install(
  DIRECTORY include/
  DESTINATION include
)

# 安装库和导出配置
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib  
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# 安装CMake导出文件
install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_package()
