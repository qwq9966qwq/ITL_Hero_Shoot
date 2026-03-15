import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bringup_dir = get_package_share_directory('bringup')
    config_file = os.path.join(bringup_dir, 'config', 'Simulation.yaml')

    lob_shot_manager_node = Node(
        package='lob_shot_manager',
        executable='lob_shot_manager_node',
        name='lob_shot_manager',
        parameters=[config_file],
        output='screen',
        remappings=[
            ('cmd_gimbal_joint', '/red_standard_robot1/cmd_gimbal_joint'),
            ('joint_states', '/red_standard_robot1/joint_states'),
            ('gimbal_world_feedback', '/red_standard_robot1/gimbal_world_feedback'),
        ],
    )

    return LaunchDescription([
        lob_shot_manager_node,
    ])
