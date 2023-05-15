#!/usr/bin/env -S python3

# MIT License
#
# Copyright (c) 2023 Naoki Takahashi
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros


def generate_launch_description():
    return launch.LaunchDescription(
        generate_declare_launch_arguments()
        + generate_launch_nodes()
    )


def generate_declare_launch_arguments():
    this_pkg_share_dir = get_package_share_directory('geometric_map_server')
    print(this_pkg_share_dir)
    return [
        launch.actions.DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace (string)'
        ),
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time (boolean)'
        ),
        launch.actions.DeclareLaunchArgument(
            'map_server_config',
            default_value=[
                os.path.join(
                    this_pkg_share_dir,
                    'config',
                    'demo_map_server.yaml'
                )
            ],
            description='Map server config file (string)'
        ),
        launch.actions.DeclareLaunchArgument(
            'map_info',
            default_value=[
                os.path.join(
                    this_pkg_share_dir,
                    'maps',
                    'demo_map_info.yaml'
                )
            ]
        )
    ]


def generate_launch_nodes():
    this_pkg_name = 'geometric_map_server'
    # this_pkg_share_dir = get_package_share_directory(this_pkg_name)
    output = 'screen'

    use_sim_time = {'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')}

    return [
        launch.actions.GroupAction(actions=[
            launch_ros.actions.PushRosNamespace(
                namespace=launch.substitutions.LaunchConfiguration('namespace')
            ),
            launch_ros.actions.Node(
                package=this_pkg_name,
                executable='geometric_map_server_node',
                name='geometric_map_server',
                output=output,
                parameters=[
                    use_sim_time,
                    {'map_info': launch.substitutions.LaunchConfiguration('map_info')},
                    launch.substitutions.LaunchConfiguration('map_server_config')
                ]
            ),
            launch_ros.actions.Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom_stf',
                output=output,
                arguments=[
                    '--frame-id', 'map',
                    '--child-frame-id', 'odom',
                    '--x', '1.5'
                ]
            ),
            launch_ros.actions.Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='odom_to_base_stf',
                output=output,
                arguments=[
                    '--frame-id', 'odom',
                    '--child-frame-id', 'base_link',
                    '--x', '1.5',
                    '--y', '2.5',
                ]
            )
        ])
    ]
