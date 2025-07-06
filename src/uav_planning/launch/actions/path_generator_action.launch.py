#!/usr/bin/env python3
"""Launch file for bioinspired path generator action server."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for path generator action server."""
    
    # Declare launch arguments
    declare_enable_px4 = DeclareLaunchArgument(
        'enable_px4',
        default_value='true',
        description='Enable PX4 integration'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for the node'
    )

    # Node for the action server
    path_generator_action_node = Node(
        package='uav_planning',
        executable='bioinspired_planner_action',
        name='bioinspired_path_generator_action_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )

    return LaunchDescription([
        declare_enable_px4,
        declare_log_level,
        path_generator_action_node,
    ])
