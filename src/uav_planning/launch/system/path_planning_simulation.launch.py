from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Launch UAV simulation with GUI monitor."""

    # Declare launch arguments
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="true",
        description="Run Gazebo in headless mode",
    )

    world_arg = DeclareLaunchArgument(
        "world", default_value="empty", description="Gazebo world to load"
    )

    monitor_delay_arg = DeclareLaunchArgument(
        "monitor_delay",
        default_value="5.0",
        description="Delay in seconds before starting the monitor",
    )

    # MicroXRCE DDS Agent for PX4 communication
    micro_xrce_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        output="screen",
        name="micro_xrce_agent",
    )

    # PX4 SITL
    px4_sitl = ExecuteProcess(
        cmd=["make", "px4_sitl", "gz_x500"],
        cwd="/workspaces/PX4-Autopilot",
        output="screen",
        name="px4_sitl",
    )

    # UAV Controller
    uav_controller = Node(
        package="uav_planning",
        executable="uav_controller",
        name="uav_controller",
        output="screen",
        parameters=[{"~/computation_time": True}],
    )

    # Bio-inspired Path Planner
    bioinspired_planner = Node(
        package="uav_planning",
        executable="bioinspired_planner_action",
        name="bioinspired_planner_action",
        output="screen",
    )

    bioinspired_planner_client = Node(
        package="uav_planning",
        executable="bioinspired_path_generator_client",
        name="bioinspired_path_generator_client",
        output="screen",
    )

    # UAV State Machine
    state_machine = Node(
        package="uav_planning",
        executable="uav_state_machine",
        name="uav_state_machine",
        output="screen",
    )

    return LaunchDescription(
        [
            # Launch arguments
            headless_arg,
            world_arg,
            monitor_delay_arg,
            # Core simulation components
            micro_xrce_agent,
            px4_sitl,
            # UAV system components
            uav_controller,
            bioinspired_planner,
            bioinspired_planner_client,
            state_machine,
            # GUI Monitor (delayed)
            # uav_monitor,
        ]
    )
