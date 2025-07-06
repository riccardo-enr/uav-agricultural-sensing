#!/usr/bin/env python3
"""
Launch file for UAV agricultural sensing simulation.
Starts Gazebo, PX4, microXRCE DDS agent, and the bioinspired path planner.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for the UAV simulation."""

    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="empty.sdf",
        description="Gazebo world file to load",
    )

    vehicle_arg = DeclareLaunchArgument(
        "vehicle", default_value="x500", description="PX4 vehicle model"
    )

    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Gazebo in headless mode",
    )

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

    # Start Gazebo with PX4 SITL using make command
    px4_cmd = ExecuteProcess(
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
            "PX4_GZ_WORLD": LaunchConfiguration("world"),
        },
    )

    # # Start PX4 SITL
    # px4_cmd = ExecuteProcess(
    #     cmd=[
    #         "/workspaces/PX4-Autopilot/build/px4_sitl_default/bin/px4",
    #         "/workspaces/PX4-Autopilot/ROMFS/px4fmu_common",
    #         "-s",
    #         "etc/init.d-posix/rcS",
    #     ],
    #     cwd="/workspaces/PX4-Autopilot",
    #     output="screen",
    #     additional_env={
    #         "PX4_SYS_AUTOSTART": "4001",  # Generic quadcopter
    #         "PX4_SIM_MODEL": LaunchConfiguration("vehicle"),
    #         "PX4_GZ_WORLD": LaunchConfiguration("world"),
    #     },
    # )

    # Start microXRCE DDS Agent (delayed to allow PX4 to start)
    microxrce_cmd = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=["MicroXRCEAgent", "udp4", "-p", "8888"], output="screen"
            )
        ],
    )

    # Start the bioinspired path planner (delayed to allow everything to initialize)
    path_planner_node = TimerAction(
        period=10.0,
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
                        "alpha": 1.5,
                        "waypoint_generation_rate": 1.0,
                        "trajectory_publish_rate": 10.0,
                        "enable_path_generation": True,
                    }
                ],
                remappings=[
                    # Remap topics if needed
                ],
            )
        ],
    )

    # Optional: Start RViz for visualization (uncomment to enable)
    # rviz_node = TimerAction(
    #     period=15.0,
    #     actions=[
    #         Node(
    #             package='rviz2',
    #             executable='rviz2',
    #             name='rviz2',
    #             output='screen',
    #             arguments=['-d', PathJoinSubstitution([
    #                 FindPackageShare('uav_planning'),
    #                 'config',
    #                 'uav_visualization.rviz'
    #             ])],
    #             condition=None  # Always start RViz, comment this line to disable
    #         )
    #     ]
    # )

    return LaunchDescription(
        [
            # Launch arguments
            world_arg,
            vehicle_arg,
            headless_arg,
            x_min_arg,
            x_max_arg,
            y_min_arg,
            y_max_arg,
            z_min_arg,
            z_max_arg,
            velocity_arg,
            visit_threshold_arg,
            # Launch processes
            px4_cmd,
            microxrce_cmd,
            path_planner_node,
            # rviz_node,  # Uncomment to enable RViz
        ]
    )
