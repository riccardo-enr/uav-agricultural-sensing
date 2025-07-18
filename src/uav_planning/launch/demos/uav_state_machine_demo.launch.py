#!/usr/bin/env python3
"""
Launch file for UAV state machine demonstration.

This launch file starts the complete system and optionally runs a demo
that shows the state machine cycling through different states.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    """Generate the launch description."""

    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="", description="Namespace for all nodes"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time"
    )

    run_demo_arg = DeclareLaunchArgument(
        "run_demo",
        default_value="true",
        description="Run the state machine demonstration",
    )

    # UAV Controller Node
    uav_controller_node = Node(
        package="uav_planning",
        executable="uav_controller",
        name="uav_controller",
        namespace=LaunchConfiguration("namespace"),
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "trajectory_publish_rate": 20.0,
            }
        ],
        output="screen",
    )

    # Path Generator Action Server Node
    path_generator_action_node = Node(
        package="uav_planning",
        executable="path_generator_action",
        name="path_generator_action_server",
        namespace=LaunchConfiguration("namespace"),
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
        output="screen",
    )

    # UAV State Machine Node
    uav_state_machine_node = Node(
        package="uav_planning",
        executable="uav_state_machine",
        name="uav_state_machine",
        namespace=LaunchConfiguration("namespace"),
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "takeoff_altitude": 5.0,
                "position_threshold": 1.0,
                "takeoff_threshold": 0.5,
                "landing_threshold": 0.3,
            }
        ],
        output="screen",
    )

    # UAV Monitor Node
    uav_monitor_node = Node(
        package="uav_planning",
        executable="uav_monitor",
        name="uav_monitor",
        namespace=LaunchConfiguration("namespace"),
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
        output="screen",
    )

    # State Machine Demo Node (delayed start)
    state_machine_demo_node = TimerAction(
        period=5.0,  # Wait 5 seconds for system to initialize
        actions=[
            Node(
                package="uav_planning",
                executable="state_machine_demo",
                name="state_machine_demo",
                namespace=LaunchConfiguration("namespace"),
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("run_demo")),
            )
        ],
    )

    return LaunchDescription(
        [
            namespace_arg,
            use_sim_time_arg,
            run_demo_arg,
            uav_controller_node,
            path_generator_action_node,
            uav_state_machine_node,
            uav_monitor_node,
            state_machine_demo_node,
        ]
    )
