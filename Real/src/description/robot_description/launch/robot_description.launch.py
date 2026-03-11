import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_description')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'sentry_real.urdf.xacro')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'visualize_robot.rviz')

    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='True', description='Launch RViz'
    )

    # joint_state_publisher_gui: 提供滑条手动调节 gimbal_yaw / gimbal_pitch
    start_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    # robot_state_publisher: 读取 URDF + joint_states -> 发布 TF
    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file])
        }],
        output='screen',
    )

    # RViz: 可视化 TF 树
    start_rviz = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        declare_rviz,
        start_joint_state_publisher_gui,
        start_robot_state_publisher,
        start_rviz,
    ])
