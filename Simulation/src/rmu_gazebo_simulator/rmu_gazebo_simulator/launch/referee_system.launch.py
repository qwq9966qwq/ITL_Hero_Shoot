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
from launch_ros.actions import Node


def generate_launch_description():
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")

    referee_config_path = os.path.join(
        pkg_simulator, "config", "referee_system_1v1.yaml"
    )

    referee_ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace="referee_system",
        arguments=[
            "/referee_system/attack_info@std_msgs/msg/String[ignition.msgs.StringMsg",
            "/referee_system/shoot_info@std_msgs/msg/String[ignition.msgs.StringMsg",
        ],
    )

    referee_ign_pose_bridge = Node(
        package="rmoss_gz_bridge",
        executable="pose_bridge",
        namespace="referee_system",
    )

    referee_ign_rfid_bridge = Node(
        package="rmoss_gz_bridge",
        executable="rfid_bridge",
        namespace="referee_system",
    )

    referee_system = Node(
        package="rmu_gazebo_simulator",
        executable="simple_competition_1v1.py",
        namespace="referee_system",
        parameters=[referee_config_path],
    )

    ld = LaunchDescription()

    ld.add_action(referee_ign_bridge)
    ld.add_action(referee_ign_pose_bridge)
    ld.add_action(referee_ign_rfid_bridge)
    ld.add_action(referee_system)

    return ld
