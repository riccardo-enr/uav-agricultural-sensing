#!/usr/bin/env python3
"""Launch file for path generator action demo with server and client."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    """Generate launch description for path generator action demo."""
    
    # Declare launch arguments
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for the nodes'
    )
    
    declare_auto_start_client = DeclareLaunchArgument(
        'auto_start_client',
        default_value='true',
        description='Automatically start the action client after server'
    )

    # Action server node
    path_generator_action_server = Node(
        package='uav_planning',
        executable='bioinspired_planner_action',
        name='bioinspired_path_generator_action_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # Action client node (delayed start to allow server to initialize)
    path_generator_action_client = TimerAction(
        period=3.0,  # Wait 3 seconds for server to start
        actions=[
            Node(
                package='uav_planning',
                executable='path_generator_client',
                name='path_generator_action_client',
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                }],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                condition=IfCondition(LaunchConfiguration('auto_start_client'))
            )
        ]
    )

    return LaunchDescription([
        declare_log_level,
        declare_auto_start_client,
        path_generator_action_server,
        path_generator_action_client,
    ])
