cmake_minimum_required(VERSION 3.5)
project(pb2025_robot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
    launch
    params
    resource
    rviz

    DESTINATION share/${PROJECT_NAME}/
)

#environment
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.dsv.in")

ament_package()
