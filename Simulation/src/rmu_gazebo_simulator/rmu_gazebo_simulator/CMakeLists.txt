cmake_minimum_required(VERSION 3.8)
project(rmu_gazebo_simulator)

find_package(ament_cmake_auto REQUIRED)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.dsv.in")

install(PROGRAMS
  scripts/referee_system/simple_competition_1v1.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  resource
  config
  launch
  rviz
)
