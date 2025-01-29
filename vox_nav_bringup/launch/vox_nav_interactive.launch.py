# Copyright (c) 2020 Fetullah Atas, Norwegian University of Life Sciences
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction

import os
import yaml

LAUNCH_PARAMS = {
    'params_file': str,
    'namespace': str,
    'use_sim_time': bool,
}


def generate_launch_description():
    launch_description_list = []
    # add launch params
    for param_name, _ in LAUNCH_PARAMS.items():
        launch_description_list.append(DeclareLaunchArgument(param_name, default_value=''))
    launch_description_list.append(OpaqueFunction(function=launch_setup))

    return LaunchDescription(launch_description_list)


def launch_setup(context, *args, **kwargs):

    share_dir = get_package_share_directory('vox_nav_bringup')

    params_file = os.path.join(share_dir, 'params', 'vox_nav_interactive_params.yaml')
    if context.launch_configurations['params_file']:
        print(f'Using params file from cli argument')
        params_file = context.launch_configurations['params_file']
    print(f'Params file path: {params_file}')

    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)
        # print(yaml.dump(params, default_flow_style=False, sort_keys=False))
        shared_params = params['/**']['ros__parameters']
        # construct params for each node
        map_server_params = params['vox_nav_interactive_map_manager']['ros__parameters']
        planner_params = params['vox_nav_planner_server_rclcpp_node']['ros__parameters']
        controller_params = params['vox_nav_controller_server_rclcpp_node']['ros__parameters']

    namespace = shared_params['namespace']

    # The planner server node creates another node, I believe get_traversability_map_client_node. If you set a name,
    # both nodes will have the same name causing a conflict.
    planner_server_node = Node(
        package='vox_nav_planning',
        executable='planner_server',
        # name='vox_nav_planner_server_rclcpp_node',
        output='screen',
        namespace=namespace,
        parameters=[shared_params, planner_params],
    )
    controller_server_node = Node(
        package='vox_nav_control',
        executable='vox_nav_controller_server',
        name='vox_nav_controller_server_rclcpp_node',
        namespace=namespace,
        output='screen',
        # prefix=['xterm -e gdb -ex run --args'],
        parameters=[shared_params, controller_params],
    )
    map_server_node = Node(
        package='vox_nav_map_server',
        executable='interactive_map_manager_node',
        name='vox_nav_interactive_map_manager',
        output='screen',
        namespace=namespace,
        # prefix=['xterm -e gdb -ex run --args'],
        parameters=[shared_params, map_server_params],
    )
    navigate_to_pose_server_node = Node(
        package='vox_nav_navigators',
        executable='navigate_to_pose_server_node',
        name='navigate_to_pose_server_node',
        namespace=namespace,
        output='screen',
        # prefix=['xterm -e gdb -ex run --args'],
        parameters=[params],
    )
    navigate_through_poses_server_node = Node(
        package='vox_nav_navigators',
        executable='navigate_through_poses_server_node',
        name='navigate_through_poses_server_node',
        namespace=namespace,
        output='screen',
        parameters=[params],
    )
    # navigate_through_gps_poses_server_node = Node(
    #     package='vox_nav_navigators',
    #     executable='navigate_through_gps_poses_server_node',
    #     name='navigate_through_gps_poses_server_node',
    #     namespace=namespace,
    #     output='screen',
    #     parameters=[params],
    # )

    return [
        planner_server_node,
        controller_server_node,
        map_server_node,
        navigate_to_pose_server_node,
        navigate_through_poses_server_node,
        # navigate_through_gps_poses_server_node
    ]
