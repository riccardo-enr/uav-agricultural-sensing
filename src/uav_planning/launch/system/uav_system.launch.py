#!/usr/bin/env python3
"""Launch file for the separated UAV controller and path generator action server."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the nodes'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Get launch configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # UAV Controller Node
    uav_controller_node = Node(
        package='uav_planning',
        executable='uav_controller',
        name='uav_controller',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            # PX4 specific remappings can be added here if needed
        ]
    )

    # Path Generator Action Server Node
    path_generator_action_node = Node(
        package='uav_planning',
        executable='path_generator_action',
        name='path_generator_action_server',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            # Action and topic remappings can be added here if needed
        ]
    )

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        uav_controller_node,
        path_generator_action_node,
    ])
