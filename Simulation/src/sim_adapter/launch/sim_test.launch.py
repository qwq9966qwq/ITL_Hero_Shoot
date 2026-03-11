"""Launch file for simulation testing with simplified URDF.

This launch file:
1. Starts the full Gazebo simulation (bringup_sim) but SKIPS its robot_state_publisher
2. Launches our own robot_state_publisher with the simplified URDF
3. Launches the joint_state_adapter to convert dual-yaw -> single-yaw

Usage:
    ros2 launch sim_adapter sim_test.launch.py

    # If robot_description package is not installed, specify URDF path:
    ros2 launch sim_adapter sim_test.launch.py \
        urdf_path:=/home/guo/Hero_Shoot/Real/src/description/robot_description/urdf/sentry_real.urdf.xacro
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # --- Paths ---
    pkg_simulator = get_package_share_directory('rmu_gazebo_simulator')
    bringup_sim_path = os.path.join(pkg_simulator, 'launch', 'bringup_sim.launch.py')

    # Locate simplified URDF: try robot_description package first, fallback to known path
    try:
        pkg_robot_desc = get_package_share_directory('robot_description')
        default_urdf = os.path.join(
            pkg_robot_desc, 'urdf', 'sentry_real.urdf.xacro'
        )
    except Exception:
        default_urdf = os.path.join(
            os.path.expanduser('~'),
            'Hero_Shoot', 'Real', 'src', 'description',
            'robot_description', 'urdf', 'sentry_real.urdf.xacro',
        )

    # --- Launch arguments ---
    declare_urdf_path = DeclareLaunchArgument(
        'urdf_path',
        default_value=default_urdf,
        description='Path to the simplified URDF xacro file',
    )

    robot_name = 'red_standard_robot1'

    # TF remapping: must match the simulation's namespace TF bus
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # --- 1. Gazebo simulation (skip its robot_state_publisher) ---
    bringup_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_sim_path),
        launch_arguments={
            'use_robot_state_publisher': 'false',
        }.items(),
    )

    # --- 2. Joint state adapter (dual-yaw -> single-yaw) ---
    joint_state_adapter = Node(
        package='sim_adapter',
        executable='joint_state_adapter',
        namespace=robot_name,
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # --- 3. Our robot_state_publisher with simplified URDF ---
    # Process xacro at launch time
    urdf_path = LaunchConfiguration('urdf_path')

    # We need the actual string for robot_description parameter,
    # so process xacro using the default path (OpaqueFunction for dynamic path)
    from launch.actions import OpaqueFunction

    # Simulation-specific dimensions (must match simulation SDF xmacro)
    sim_xacro_mappings = {
        'chassis_height':      '0.063',
        'gimbal_yaw_height':   '0.1376',
        'gimbal_pitch_height': '0.172',
        'muzzle_forward':      '0.15',
        'mid360_x':     '0.16',
        'mid360_y':     '0.0',
        'mid360_z':     '0.18',
        'mid360_roll':  '0.0',
        'mid360_pitch': '0.2618',  # pi/12 ≈ 15° (simulation)
        'mid360_yaw':   '0.0',
    }

    def launch_robot_state_publisher(context):
        urdf_file = context.launch_configurations['urdf_path']
        robot_description_xml = xacro.process_file(
            urdf_file, mappings=sim_xacro_mappings
        ).toxml()

        return [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=robot_name,
                remappings=remappings + [
                    ('joint_states', 'adapted_joint_states'),
                ],
                parameters=[
                    {
                        'use_sim_time': True,
                        'robot_description': robot_description_xml,
                    }
                ],
            ),
        ]

    robot_state_publisher_action = OpaqueFunction(function=launch_robot_state_publisher)

    # --- Assemble ---
    ld = LaunchDescription()
    ld.add_action(declare_urdf_path)
    ld.add_action(bringup_sim)
    ld.add_action(joint_state_adapter)
    ld.add_action(robot_state_publisher_action)

    return ld
