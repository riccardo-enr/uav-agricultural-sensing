#!/usr/bin/env python3
"""
Demo launch file showing different usage scenarios.
This file demonstrates various ways to launch the UAV simulation.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate demo launch configurations."""

    # Get package directory
    pkg_dir = get_package_share_directory("uav_planning")

    # Launch mode selection
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="planner_only",
        choices=["planner_only", "full_simulation", "agricultural_survey"],
        description="Launch mode: planner_only, full_simulation, or agricultural_survey",
    )

    # Common parameters
    enable_logging_arg = DeclareLaunchArgument(
        "enable_logging",
        default_value="true",
        description="Enable detailed logging",
    )

    # Mode 1: Path planner only (for when PX4 is already running)
    planner_only = GroupAction(
        condition=IfCondition("$(eval '$(var mode)' == 'planner_only')"),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(pkg_dir, "launch", "path_planner.launch.py")]
                ),
                launch_arguments={
                    "x_min": "-15.0",
                    "x_max": "15.0",
                    "y_min": "-15.0",
                    "y_max": "15.0",
                    "z_min": "5.0",
                    "z_max": "12.0",
                    "velocity": "6.0",
                    "visit_threshold": "1.5",
                }.items(),
            )
        ],
    )

    # Mode 2: Full simulation
    full_simulation = GroupAction(
        condition=IfCondition("$(eval '$(var mode)' == 'full_simulation')"),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            pkg_dir, "launch", "uav_simulation.launch.py"
                        )
                    ]
                ),
                launch_arguments={
                    "world": "empty.sdf",
                    "vehicle": "x500",
                    "headless": "false",
                    "x_min": "-25.0",
                    "x_max": "25.0",
                    "y_min": "-25.0",
                    "y_max": "25.0",
                    "z_min": "8.0",
                    "z_max": "20.0",
                    "velocity": "10.0",
                    "visit_threshold": "2.5",
                }.items(),
            )
        ],
    )

    # Mode 3: Agricultural survey (optimized for large area coverage)
    agricultural_survey = GroupAction(
        condition=IfCondition("$(eval '$(var mode)' == 'agricultural_survey')"),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(pkg_dir, "launch", "path_planner.launch.py")]
                ),
                launch_arguments={
                    "x_min": "-100.0",
                    "x_max": "100.0",
                    "y_min": "-75.0",
                    "y_max": "75.0",
                    "z_min": "15.0",
                    "z_max": "30.0",
                    "velocity": "15.0",
                    "visit_threshold": "5.0",
                }.items(),
            )
        ],
    )

    return LaunchDescription(
        [
            mode_arg,
            enable_logging_arg,
            planner_only,
            full_simulation,
            agricultural_survey,
        ]
    )
