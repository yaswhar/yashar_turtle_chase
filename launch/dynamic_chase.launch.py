#!/usr/bin/env python3

# Copyright 2025 Yashar Zafari
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law of or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'num_turtles',
            default_value='2',
            description='Number of turtles to spawn'
        ),
        DeclareLaunchArgument(
            'initial_leader',
            default_value='turtle1',
            description='Initial leader turtle name'
        ),

        # Turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),

        # Spawn turtles node
        Node(
            package='yashar_turtle_chase',
            executable='spawn_turtles',
            name='spawn_turtles',
            parameters=[{'num_turtles': LaunchConfiguration('num_turtles')}]
        ),

        # Dynamic follower node
        Node(
            package='yashar_turtle_chase',
            executable='dynamic_follower',
            name='dynamic_follower',
            parameters=[
                {'num_turtles': LaunchConfiguration('num_turtles')},
                {'initial_leader': LaunchConfiguration('initial_leader')}
            ]
        ),

        # Teleop launch command
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'turtlesim', 'turtle_teleop_key',
                '--ros-args',
                '-r', '/turtle1/cmd_vel:=/leader/cmd_vel'
            ],
            output='screen',
            shell=True,
            prefix='xterm -e'
        )
    ])
