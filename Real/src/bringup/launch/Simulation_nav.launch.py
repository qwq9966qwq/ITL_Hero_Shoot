import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('bringup')

    default_config = os.path.join(bringup_dir, 'config', 'Simulation.yaml')
    default_pcd = os.path.join(bringup_dir, 'pcd', 'Hero.pcd')
    rviz_cfg_path = os.path.join(bringup_dir, 'rviz', 'nav.rviz')

    # 仿真器的 robot_state_publisher 在 namespace red_standard_robot1 下，
    # TF 发布在 /red_standard_robot1/tf 和 /red_standard_robot1/tf_static。
    # 必须把我们的节点也 remap 到同一个 TF bus，否则看不到机器人 URDF 的 TF 链。
    remappings = [
        ('/tf', '/red_standard_robot1/tf'),
        ('/tf_static', '/red_standard_robot1/tf_static'),
    ]

    # Launch arguments
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='仿真配置文件路径',
    )

    declare_pcd_file = DeclareLaunchArgument(
        'pcd_file',
        default_value=default_pcd,
        description='先验 PCD 地图路径',
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='是否启动 RViz',
    )

    # Point-LIO 里程计（导航模式：关闭 PCD 保存，禁用 TF 发布避免孤立帧）
    start_point_lio = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='point_lio',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': True,
                'pcd_save.pcd_save_en': False,
                'publish.tf_send_en': False,
            },
        ],
        remappings=remappings,
        output='screen',
    )

    # loam_adapter 坐标适配（Point_LIO → 标准 TF）
    start_loam_adapter = Node(
        package='loam_adapter',
        executable='loam_adapter_node',
        name='loam_adapter',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': True},
        ],
        remappings=remappings,
        output='screen',
    )

    # relocalization 重定位（GICP 配准先验地图，发布 map→odom）
    start_relocalization = Node(
        package='relocalization',
        executable='relocalization_node',
        name='relocalization',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': True,
                'prior_pcd_file': LaunchConfiguration('pcd_file'),
            },
        ],
        remappings=remappings,
        output='screen',
    )

    # RViz 导航可视化
    start_rviz = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg_path],
        parameters=[{'use_sim_time': True}],
        remappings=remappings,
    )

    # 吊射求解器
    start_lob_shot = Node(
        package='lob_shot_manager',
        executable='lob_shot_manager_node',
        name='lob_shot_manager',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': True},
        ],
        remappings=remappings + [
            ('cmd_gimbal_joint', '/red_standard_robot1/cmd_gimbal_joint'),
            ('joint_states', '/red_standard_robot1/adapted_joint_states'),
        ],
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(declare_config_file)
    ld.add_action(declare_pcd_file)
    ld.add_action(declare_rviz)
    ld.add_action(start_point_lio)
    ld.add_action(start_loam_adapter)
    ld.add_action(start_relocalization)
    ld.add_action(start_lob_shot)
    ld.add_action(start_rviz)

    return ld
