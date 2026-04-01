#!/usr/bin/env python3
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_share = get_package_share_directory('franka_bringup')


def load_yaml(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f'File not found: {file_path}')
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)


def generate_robot_nodes(context):
    config_file = LaunchConfiguration('robot_config_file').perform(context)
    if not os.path.isabs(config_file) and os.path.sep not in config_file:
        config_file = os.path.join(package_share, 'config', config_file)

    configs = load_yaml(config_file)
    nodes = []

    for _, config in configs.items():
        namespace = str(config.get('namespace', ''))
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py']
                    )
                ),
                launch_arguments={
                    'arm_id': str(config.get('arm_id', config.get('robot_type', 'fr3'))),
                    'arm_prefix': str(config.get('arm_prefix', '')),
                    'namespace': namespace,
                    'urdf_file': str(config.get('urdf_file', 'fr3/fr3.urdf.xacro')),
                    'robot_ip': str(config['robot_ip']),
                    'load_gripper': str(config.get('load_gripper', 'false')),
                    'use_fake_hardware': str(config.get('use_fake_hardware', 'false')),
                    'fake_sensor_commands': str(config.get('fake_sensor_commands', 'false')),
                    'joint_state_rate': str(config.get('joint_state_rate', 30)),
                }.items(),
            )
        )

        nodes.append(
            Node(
                package='controller_manager',
                executable='spawner',
                namespace=namespace,
                arguments=['gravity_compensation_example_controller', '--controller-manager-timeout', '30'],
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare('franka_bringup'), 'config', 'controllers.yaml']
                    )
                ],
                output='screen',
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config_file',
            default_value='franka.config.yaml',
            description='Config file name (looked up in franka_bringup/config/) or full path',
        ),
        OpaqueFunction(function=generate_robot_nodes),
    ])
