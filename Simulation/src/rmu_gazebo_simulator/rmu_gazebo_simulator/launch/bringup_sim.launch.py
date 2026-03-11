# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")

    gz_world_path = os.path.join(pkg_simulator, "config", "gz_world.yaml")
    with open(gz_world_path) as file:
        config = yaml.safe_load(file)
        selected_world = config.get("world")

    world_sdf_path = os.path.join(
        pkg_simulator, "resource", "worlds", f"{selected_world}_world.sdf"
    )
    ign_config_path = os.path.join(pkg_simulator, "resource", "ign", "gui.config")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world_sdf_path": world_sdf_path,
            "ign_config_path": ign_config_path,
        }.items(),
    )

    # Optional: forward use_robot_state_publisher to spawn_robots
    declare_use_rsp = DeclareLaunchArgument(
        'use_robot_state_publisher',
        default_value='true',
        description='Forward to spawn_robots: launch auto-generated robot_state_publisher',
    )

    spawn_robots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "spawn_robots.launch.py")
        ),
        launch_arguments={
            'use_robot_state_publisher': LaunchConfiguration('use_robot_state_publisher'),
        }.items(),
    )

    referee_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "referee_system.launch.py")
        )
    )

    cloud_converter_config = os.path.join(
        pkg_simulator, "config", "cloud_converter.yaml"
    )
    pointcloud_converter_node = Node(
        package="ign_sim_pointcloud_tool",
        executable="ign_sim_pointcloud_tool_node",
        name="ign_sim_pointcloud_tool",
        parameters=[cloud_converter_config],
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_rsp)
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_robots_launch)
    ld.add_action(referee_system_launch)
    ld.add_action(pointcloud_converter_node)

    return ld
