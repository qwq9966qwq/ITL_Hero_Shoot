import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('bringup')
    point_lio_dir = get_package_share_directory('point_lio')

    default_config = os.path.join(bringup_dir, 'config', 'Simulation.yaml')
    rviz_cfg_path = os.path.join(point_lio_dir, 'rviz_cfg', 'loam_livox.rviz')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Point-LIO 仿真配置文件路径',
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='是否启动 RViz 可视化',
    )

    # Point-LIO 建图节点，启用 use_sim_time 和 PCD 保存
    start_point_lio_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='point_lio',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': True,
                'pcd_save.pcd_save_en': True,
                'pcd_save.interval': -1,
            },
        ],
        remappings=remappings,
        output='screen',
    )

    start_rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg_path],
        parameters=[{'use_sim_time': True}],
        remappings=remappings,
    )

    ld = LaunchDescription()

    ld.add_action(declare_config_file)
    ld.add_action(declare_rviz)
    ld.add_action(start_point_lio_node)
    ld.add_action(start_rviz_node)

    return ld
