#!/usr/bin/env python3
"""Launch file for UAV system with path generator demo."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
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

    auto_start_demo_arg = DeclareLaunchArgument(
        'auto_start_demo',
        default_value='true',
        description='Automatically start the demo client'
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
        }]
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
        }]
    )

    # Demo Client Node (delayed start to allow action server to initialize)
    demo_client_node = TimerAction(
        period=3.0,  # Wait 3 seconds for action server to start
        actions=[
            Node(
                package='uav_planning',
                executable='path_generator_client',
                name='path_generator_demo_client',
                namespace=namespace,
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'x_min': -10.0,
                    'x_max': 10.0,
                    'y_min': -10.0,
                    'y_max': 10.0,
                    'z_min': 1.0,
                    'z_max': 5.0,
                    'alpha': 2.0,
                    'visit_threshold': 2.0,
                    'max_waypoints': 20,
                    'exploration_time': 120.0,
                    'enable_auto_arm': False,  # Set to true for real flights
                }],
                condition=LaunchConfiguration('auto_start_demo')
            )
        ]
    )

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        auto_start_demo_arg,
        uav_controller_node,
        path_generator_action_node,
        demo_client_node,
    ])
