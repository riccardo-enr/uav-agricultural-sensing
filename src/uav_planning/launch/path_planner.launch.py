#!/usr/bin/env python3

"""
Simple launch file for the bioinspired path planner with microXRCE DDS agent.
Use this when PX4 and Gazebo are already running.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for the path planner only."""

    # Path planner parameters
    x_min_arg = DeclareLaunchArgument(
        "x_min",
        default_value="-20.0",
        description="Minimum X coordinate for exploration area",
    )

    x_max_arg = DeclareLaunchArgument(
        "x_max",
        default_value="20.0",
        description="Maximum X coordinate for exploration area",
    )

    y_min_arg = DeclareLaunchArgument(
        "y_min",
        default_value="-20.0",
        description="Minimum Y coordinate for exploration area",
    )

    y_max_arg = DeclareLaunchArgument(
        "y_max",
        default_value="20.0",
        description="Maximum Y coordinate for exploration area",
    )

    z_min_arg = DeclareLaunchArgument(
        "z_min",
        default_value="5.0",
        description="Minimum Z coordinate for exploration area",
    )

    z_max_arg = DeclareLaunchArgument(
        "z_max",
        default_value="15.0",
        description="Maximum Z coordinate for exploration area",
    )

    velocity_arg = DeclareLaunchArgument(
        "velocity",
        default_value="8.0",
        description="UAV cruise velocity in m/s",
    )

    visit_threshold_arg = DeclareLaunchArgument(
        "visit_threshold",
        default_value="2.0",
        description="Distance threshold to consider waypoint reached",
    )

    enable_path_generation_arg = DeclareLaunchArgument(
        "enable_path_generation",
        default_value="true",
        description="Enable automatic path generation",
    )

    # # Start microXRCE DDS Agent
    # microxrce_cmd = ExecuteProcess(
    #     cmd=["MicroXRCEAgent", "udp4", "-p", "8888"], output="screen"
    # )

    # Start the bioinspired path planner (delayed to allow microXRCE to start)
    path_planner_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="uav_planning",
                executable="bioinspired_planner",
                name="bioinspired_path_generator",
                output="screen",
                parameters=[
                    {
                        "x_min": LaunchConfiguration("x_min"),
                        "x_max": LaunchConfiguration("x_max"),
                        "y_min": LaunchConfiguration("y_min"),
                        "y_max": LaunchConfiguration("y_max"),
                        "z_min": LaunchConfiguration("z_min"),
                        "z_max": LaunchConfiguration("z_max"),
                        "velocity": LaunchConfiguration("velocity"),
                        "visit_threshold": LaunchConfiguration(
                            "visit_threshold"
                        ),
                        "enable_path_generation": LaunchConfiguration(
                            "enable_path_generation"
                        ),
                        "alpha": 1.5,
                        "waypoint_generation_rate": 1.0,
                        "trajectory_publish_rate": 10.0,
                    }
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            # Launch arguments
            x_min_arg,
            x_max_arg,
            y_min_arg,
            y_max_arg,
            z_min_arg,
            z_max_arg,
            velocity_arg,
            visit_threshold_arg,
            enable_path_generation_arg,
            # Launch processes
            # microxrce_cmd,
            path_planner_node,
        ]
    )
