#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess

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
        
        # Dynamic follower node (fixed in code)
        Node(
            package='yashar_turtle_chase',
            executable='dynamic_follower',
            name='dynamic_follower',
            parameters=[
                {'num_turtles': LaunchConfiguration('num_turtles')},
                {'initial_leader': LaunchConfiguration('initial_leader')}
            ]
        ),
        
        # Fixed teleop launch command
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'turtlesim', 'turtle_teleop_key',  # Fixed package name
                '--ros-args',
                '-r', '/turtle1/cmd_vel:=/leader/cmd_vel'
            ],
            output='screen',
            shell=True,
            prefix='xterm -e'
        )
    ])