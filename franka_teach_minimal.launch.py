#!/usr/bin/env python3
import os
import sys

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_share = get_package_share_directory('franka_bringup')
utils_path = os.path.join(package_share, '..', '..', 'lib', 'franka_bringup', 'utils')
sys.path.append(os.path.abspath(utils_path))

from launch_utils import load_yaml  # noqa: E402


def generate_robot_nodes(context):
    config_file = LaunchConfiguration('robot_config_file').perform(context)
    configs = load_yaml(config_file)
    nodes = []
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('franka_bringup'), 'config', 'controllers.yaml'
    ]).perform(context)

    for _, config in configs.items():
        namespace = str(config['namespace'])
        load_gripper = str(config['load_gripper'])
        urdf_path = PathJoinSubstitution([
            FindPackageShare('franka_description'), 'robots', str(config['urdf_file'])
        ]).perform(context)
        robot_description = xacro.process_file(
            urdf_path,
            mappings={
                'ros2_control': 'true',
                'arm_id': str(config['arm_id']),
                'arm_prefix': str(config['arm_prefix']),
                'robot_ip': str(config['robot_ip']),
                'hand': load_gripper,
                'use_fake_hardware': str(config['use_fake_hardware']),
                'fake_sensor_commands': str(config['fake_sensor_commands']),
            }
        ).toprettyxml(indent='  ')

        nodes.extend([
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=namespace,
                parameters=[{'robot_description': robot_description}],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                namespace=namespace,
                parameters=[
                    controllers_yaml,
                    {'robot_description': robot_description},
                    {'load_gripper': load_gripper.lower() == 'true'},
                ],
                remappings=[('joint_states', 'franka/joint_states')],
                output='screen',
            ),
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                namespace=namespace,
                parameters=[{
                    'source_list': ['franka/joint_states', 'franka_gripper/joint_states'],
                    'rate': int(config['joint_state_rate']),
                    'use_robot_description': False,
                }],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                namespace=namespace,
                arguments=['joint_state_broadcaster'],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                namespace=namespace,
                arguments=['gravity_compensation_example_controller', '--controller-manager-timeout', '30'],
                parameters=[controllers_yaml],
                output='screen',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([
                    FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'
                ])]),
                launch_arguments={
                    'namespace': namespace,
                    'robot_ip': str(config['robot_ip']),
                    'use_fake_hardware': str(config['use_fake_hardware']),
                    'arm_id': str(config['arm_id']),
                }.items(),
                condition=IfCondition(load_gripper),
            ),
        ])

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('franka_bringup'), 'config', 'franka.config.yaml'
            ]),
            description='Path to the robot configuration file to load',
        ),
        OpaqueFunction(function=generate_robot_nodes),
    ])
