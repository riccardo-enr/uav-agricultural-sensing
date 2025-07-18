#!/usr/bin/env python3
"""
Launch file for the complete UAV system with state machine.

This launch file starts:
1. UAV Controller - Low-level vehicle control
2. Path Generator Action Server - Mission planning
3. UAV State Machine - High-level state management
4. UAV Monitor - System monitoring
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate the launch description."""

    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="", description="Namespace for all nodes"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time"
    )

    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Gazebo in headless mode",
    )

    # Start PX4 SITL with Gazebo integration
    px4_cmd = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "make",
                    "px4_sitl",
                    "gz_px4vision",
                ],
                cwd="/workspaces/PX4-Autopilot",
                output="screen",
                additional_env={
                    "HEADLESS": "1"
                    if LaunchConfiguration("headless") == "true"
                    else "0",
                },
            )
        ],
    )

    # Start microXRCE DDS Agent
    microxrce_cmd = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=["MicroXRCEAgent", "udp4", "-p", "8888"], output="screen"
            )
        ],
    )

    # UAV Controller Node
    uav_controller_node = TimerAction(
        period=12.0,
        actions=[
            Node(
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
        ],
    )

    # Path Generator Action Server Node
    path_generator_action_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package="uav_planning",
                executable="bioinspired_planner_action",
                name="bioinspired_planner_action_server",
                namespace=LaunchConfiguration("namespace"),
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
                output="screen",
            )
        ],
    )

    # UAV State Machine Node
    uav_state_machine_node = TimerAction(
        period=15.0,
        actions=[
            Node(
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
        ],
    )

    # UAV Monitor Node
    uav_monitor_node = TimerAction(
        period=12.0,
        actions=[
            Node(
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
        ],
    )

    return LaunchDescription(
        [
            # Launch arguments
            namespace_arg,
            use_sim_time_arg,
            headless_arg,
            # Simulation processes
            px4_cmd,
            microxrce_cmd,
            # UAV system nodes
            uav_controller_node,
            path_generator_action_node,
            uav_state_machine_node,
            uav_monitor_node,
        ]
    )
