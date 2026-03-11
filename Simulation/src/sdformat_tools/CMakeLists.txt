cmake_minimum_required(VERSION 3.5)
project(sdformat_tools)

find_package(ament_cmake REQUIRED)

# install sdformat_tools python package
ament_python_install_package(sdformat_tools)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
