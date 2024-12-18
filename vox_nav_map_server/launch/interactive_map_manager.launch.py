import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace

import yaml


def generate_launch_description():
    share_dir = get_package_share_directory('vox_nav_map_server')

    params_file = os.path.join(share_dir, 'config', 'interactive_map_manager.yaml')

    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['interactive_map_manager']['ros__parameters']

    return LaunchDescription([
        Node(
            package='vox_nav_map_server',
            executable='interactive_map_manager_node',
            name='interactive_map_manager',
            namespace=params['namespace'],
            parameters=[params],
            output='screen'
        ),
    ])
