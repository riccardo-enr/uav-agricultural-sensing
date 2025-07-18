#!/usr/bin/env python3
"""
Launch file for the UAV state machine system demonstration.
This launch file starts all the core components needed for state machine operation.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for UAV state machine system."""

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time"
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="info", description="Log level for nodes"
    )

    # Launch configuration
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    # Node definitions
    state_machine_node = Node(
        package="uav_planning",
        executable="uav_state_machine",
        name="uav_state_machine",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    controller_node = Node(
        package="uav_planning",
        executable="uav_controller",
        name="uav_controller",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    path_generator_action_node = Node(
        package="uav_planning",
        executable="bioinspired_planner_action",
        name="bioinspired_path_generator_action_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Optional: Monitor node for debugging
    monitor_node = Node(
        package="uav_planning",
        executable="uav_monitor",
        name="uav_monitor",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription(
        [
            # Launch arguments
            use_sim_time_arg,
            log_level_arg,
            # Core system nodes
            state_machine_node,
            controller_node,
            path_generator_action_node,
            # Optional monitoring
            monitor_node,
        ]
    )
