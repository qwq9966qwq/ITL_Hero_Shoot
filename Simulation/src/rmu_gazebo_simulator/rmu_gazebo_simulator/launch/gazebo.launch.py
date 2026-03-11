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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")

    world_sdf_path = LaunchConfiguration("world_sdf_path")
    ign_config_path = LaunchConfiguration("ign_config_path")

    declare_world_sdf_path = DeclareLaunchArgument(
        "world_sdf_path",
        default_value=os.path.join(
            pkg_simulator, "resource", "worlds", "rmul_2024_world.sdf"
        ),
        description="Path to the world SDF file",
    )

    declare_ign_config_path = DeclareLaunchArgument(
        "ign_config_path",
        default_value=os.path.join(pkg_simulator, "resource", "ign", "gui.config"),
        description="Path to the Ignition Gazebo GUI configuration file",
    )

    # Launch Gazebo simulator
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_version": "6",
            "gz_args": [
                world_sdf_path,
                TextSubstitution(text=" --gui-config "),
                ign_config_path,
            ],
        }.items(),
    )

    robot_ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_world_sdf_path)
    ld.add_action(declare_ign_config_path)
    ld.add_action(gazebo)
    ld.add_action(robot_ign_bridge)

    return ld
