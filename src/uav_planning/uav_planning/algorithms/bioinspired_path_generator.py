#!/usr/bin/env python3
"""Bio-inspired path generator ROS 2 node using Levy flight patterns."""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

try:
    from px4_msgs.msg import (
        TrajectorySetpoint,
        VehicleOdometry,
        OffboardControlMode,
        VehicleCommand,
        VehicleStatus,
    )
except ImportError:
    # Fallback if px4_msgs not available
    TrajectorySetpoint = None
    VehicleOdometry = None
    OffboardControlMode = None
    VehicleCommand = None
    VehicleStatus = None
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Int32
import numpy as np
from .butterfly import TimeBasedButterflyExplorer as ButterflyExplorer


class BioinspiredPathGenerator(Node):
    """
    ROS 2 node that generates bio-inspired flight paths using Levy flight patterns
    and publishes trajectory setpoints compatible with PX4 autopilot.
    """

    def __init__(self):
        super().__init__("bioinspired_path_generator")

        # Check if PX4 messages are available
        if (
            TrajectorySetpoint is None
            or VehicleCommand is None
            or VehicleStatus is None
        ):
            self.get_logger().warn(
                "px4_msgs not available or incomplete. Install px4_msgs package for PX4 compatibility."
            )
            self.px4_available = False
        else:
            self.px4_available = True

        # Declare parameters with default values
        self.declare_parameter("x_min", -10.0)
        self.declare_parameter("x_max", 10.0)
        self.declare_parameter("y_min", -10.0)
        self.declare_parameter("y_max", 10.0)
        self.declare_parameter("z_min", 2.0)
        self.declare_parameter("z_max", 8.0)
        self.declare_parameter("alpha", 1.5)
        self.declare_parameter("visit_threshold", 1.2)
        self.declare_parameter("waypoint_generation_rate", 1.0)  # Hz
        self.declare_parameter("trajectory_publish_rate", 10.0)  # Hz
        self.declare_parameter("uav_speed_limit", 0.5)  # m/s
        self.declare_parameter("velocity", 5.0)  # m/s
        self.declare_parameter("enable_path_generation", True)
        # Add time-based exploration parameters
        self.declare_parameter("exploration_duration", 600.0)  # seconds
        self.declare_parameter("drone_speed", 0.5)  # m/s

        # Get parameters
        self._load_parameters()

        # Initialize butterfly explorer
        self._initialize_explorer()

        # QoS profile for PX4 compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        if self.px4_available:
            self.trajectory_setpoint_pub = self.create_publisher(
                TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile
            )

            self.offboard_control_mode_pub = self.create_publisher(
                OffboardControlMode,
                "/fmu/in/offboard_control_mode",
                qos_profile,
            )

            if VehicleCommand is not None:
                self.vehicle_command_pub = self.create_publisher(
                    VehicleCommand,
                    "/fmu/in/vehicle_command",
                    qos_profile,
                )
            else:
                self.vehicle_command_pub = None

            # Subscribers
            self.vehicle_odometry_sub = self.create_subscription(
                VehicleOdometry,
                "/fmu/out/vehicle_odometry",
                self.vehicle_odometry_callback,
                qos_profile,
            )

            if VehicleStatus is not None:
                self.vehicle_status_sub = self.create_subscription(
                    VehicleStatus,
                    "/fmu/out/vehicle_status_v1",
                    self.vehicle_status_callback,
                    qos_profile,
                )
            else:
                self.vehicle_status_sub = None
        else:
            self.trajectory_setpoint_pub = None
            self.offboard_control_mode_pub = None
            self.vehicle_command_pub = None
            self.vehicle_odometry_sub = None
            self.vehicle_status_sub = None

        # Debug publishers using node-specific topics
        self.current_waypoint_pub = self.create_publisher(
            Point, "~/current_waypoint", 10
        )

        self.computation_time_pub = self.create_publisher(
            Float64, "~/computation_time", 10
        )

        self.corners_remaining_pub = self.create_publisher(
            Int32, "/debug/corners_remaining", 10
        )

        self.exploration_time_pub = self.create_publisher(
            Float64, "/debug/exploration_time", 10
        )

        self.completion_rate_pub = self.create_publisher(
            Float64, "/debug/completion_rate", 10
        )

        self.time_remaining_pub = self.create_publisher(
            Float64, "/debug/time_remaining", 10
        )

        # State variables
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_waypoint = None
        self.waypoint_reached = True
        self.vehicle_position_valid = False
        self.exploration_start_time = None

        # UAV state variables
        self.vehicle_armed = False
        self.offboard_mode_active = False
        self.nav_state = 0
        self.arming_state = 0
        self.offboard_setpoint_counter = 0

        # Timers
        self.fallback_waypoint_timer = self.create_timer(
            5.0,
            self.fallback_waypoint_generation,
        )

        self.trajectory_timer = self.create_timer(
            1.0 / self.trajectory_publish_rate, self.publish_trajectory_callback
        )

        self.offboard_timer = self.create_timer(
            0.1,  # 10 Hz
            self.publish_offboard_control_mode,
        )

        # Generate initial waypoint
        if self.enable_path_generation:
            self.generate_next_waypoint()

        # Initialize state tracking variables
        self._last_arming_state = None
        self._last_nav_state = None

        self.get_logger().info("Bioinspired path generator initialized")
        self.get_logger().info(
            f"Bounding box: X[{self.x_min}, {self.x_max}], "
            f"Y[{self.y_min}, {self.y_max}], Z[{self.z_min}, {self.z_max}]"
        )

        if self.px4_available:
            self.get_logger().info("Automatic arming and offboard mode enabled")

    def _load_parameters(self):
        """Load ROS parameters."""
        self.x_min = (
            self.get_parameter("x_min").get_parameter_value().double_value
        )
        self.x_max = (
            self.get_parameter("x_max").get_parameter_value().double_value
        )
        self.y_min = (
            self.get_parameter("y_min").get_parameter_value().double_value
        )
        self.y_max = (
            self.get_parameter("y_max").get_parameter_value().double_value
        )
        self.z_min = (
            self.get_parameter("z_min").get_parameter_value().double_value
        )
        self.z_max = (
            self.get_parameter("z_max").get_parameter_value().double_value
        )
        self.alpha = (
            self.get_parameter("alpha").get_parameter_value().double_value
        )
        self.visit_threshold = (
            self.get_parameter("visit_threshold")
            .get_parameter_value()
            .double_value
        )
        self.waypoint_generation_rate = (
            self.get_parameter("waypoint_generation_rate")
            .get_parameter_value()
            .double_value
        )
        self.trajectory_publish_rate = (
            self.get_parameter("trajectory_publish_rate")
            .get_parameter_value()
            .double_value
        )
        self.velocity = (
            self.get_parameter("velocity").get_parameter_value().double_value
        )
        self.enable_path_generation = (
            self.get_parameter("enable_path_generation")
            .get_parameter_value()
            .bool_value
        )
        # Add exploration duration parameter
        self.exploration_duration = (
            self.get_parameter("exploration_duration")
            .get_parameter_value()
            .double_value
        )
        self.drone_speed = (
            self.get_parameter("drone_speed").get_parameter_value().double_value
        )

    def _initialize_explorer(self):
        """Initialize the butterfly explorer with current parameters."""
        self.explorer = ButterflyExplorer(
            x_min=self.x_min,
            x_max=self.x_max,
            y_min=self.y_min,
            y_max=self.y_max,
            z_min=self.z_min,
            z_max=self.z_max,
            alpha=self.alpha,
            visit_threshold=self.visit_threshold,
            drone_speed=self.drone_speed,
        )

    # ========================
    # CALLBACK FUNCTIONS
    # ========================

    def vehicle_odometry_callback(self, msg):
        """Update current vehicle position from odometry and generate next waypoint if needed."""
        self.current_position = np.array(
            [
                msg.position[1],
                msg.position[0],
                -msg.position[2],  # PX4 uses NED, convert to positive up
            ]
        )
        self.vehicle_position_valid = True

        # Initialize exploration start time
        if self.exploration_start_time is None:
            self.exploration_start_time = self.get_clock().now()

        # Update explorer's current position and time
        if hasattr(self.explorer, "current_pos"):
            distance_traveled = np.linalg.norm(
                self.current_position - self.explorer.current_pos
            )
            self.explorer.total_distance_traveled += float(distance_traveled)
            self.explorer.current_pos = self.current_position

            # Update time in explorer
            elapsed_time = (
                self.get_clock().now() - self.exploration_start_time
            ).nanoseconds / 1e9
            self.explorer.current_time = elapsed_time

        # Check if current waypoint is reached and generate next one
        if self.current_waypoint is not None:
            distance = np.linalg.norm(
                self.current_position - self.current_waypoint
            )
            if distance < self.visit_threshold:
                if not self.waypoint_reached:
                    self.get_logger().info(
                        f"Waypoint reached! Distance: {distance:.2f}m, "
                        f"Position: [{self.current_position[0]:.2f}, "
                        f"{self.current_position[1]:.2f}, {self.current_position[2]:.2f}]"
                    )
                self.waypoint_reached = True
                if self.enable_path_generation:
                    self.generate_next_waypoint()
        elif self.enable_path_generation:
            self.generate_next_waypoint()

    def vehicle_status_callback(self, msg):
        """Update vehicle status information for arming and mode monitoring."""
        self.vehicle_armed = msg.arming_state == 2  # ARMING_STATE_ARMED
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

        self.offboard_mode_active = msg.nav_state == 14  # NAV_STATE_OFFBOARD

        if (
            hasattr(self, "_last_arming_state")
            and self._last_arming_state != msg.arming_state
        ):
            arming_states = {
                1: "DISARMED",
                2: "ARMED",
            }
            self.get_logger().info(
                f"Arming state changed: {arming_states.get(msg.arming_state, f'UNKNOWN({msg.arming_state})')}"
            )
        self._last_arming_state = msg.arming_state

        if (
            hasattr(self, "_last_nav_state")
            and self._last_nav_state != msg.nav_state
        ):
            self.get_logger().info(f"Navigation state changed: {msg.nav_state}")
        self._last_nav_state = msg.nav_state

    def fallback_waypoint_generation(self):
        """Fallback waypoint generation when odometry is not available."""
        if not self.vehicle_position_valid and self.enable_path_generation:
            self.get_logger().warn(
                "No odometry available, using fallback waypoint generation"
            )
            self.generate_next_waypoint()

    def generate_waypoint_callback(self):
        """Legacy callback - now called generate_next_waypoint for clarity."""
        self.generate_next_waypoint()

    # ========================
    # PUBLISHER FUNCTIONS
    # ========================

    def publish_trajectory_callback(self):
        """Publish trajectory setpoint for PX4."""
        if (
            self.current_waypoint is None
            or not self.px4_available
            or TrajectorySetpoint is None
        ):
            return

        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        msg.position[0] = float(self.current_waypoint[1])  # NED frame for PX4
        msg.position[1] = float(self.current_waypoint[0])
        msg.position[2] = -float(self.current_waypoint[2])

        msg.velocity[0] = float("nan")  # Let PX4 handle trajectory generation
        msg.velocity[1] = float("nan")
        msg.velocity[2] = float("nan")

        if self.vehicle_position_valid and self.current_waypoint is not None:
            direction = self.current_waypoint - self.current_position
            if np.linalg.norm(direction[:2]) > 0.1:
                msg.yaw = float(np.arctan2(direction[1], direction[0]))
            else:
                msg.yaw = float("nan")
        else:
            msg.yaw = float("nan")

        if self.trajectory_setpoint_pub is not None:
            self.trajectory_setpoint_pub.publish(msg)

    def publish_offboard_control_mode(self):
        """Publish offboard control mode for PX4 and handle auto arming."""
        if not self.px4_available or OffboardControlMode is None:
            return

        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        if self.offboard_control_mode_pub is not None:
            self.offboard_control_mode_pub.publish(msg)

        self.auto_arm_and_offboard()

    def publish_debug_info(self, waypoint, computation_time):
        """Publish debug information for waypoint generation."""
        waypoint_msg = Point()
        waypoint_msg.x = float(waypoint[0])
        waypoint_msg.y = float(waypoint[1])
        waypoint_msg.z = float(waypoint[2])
        self.current_waypoint_pub.publish(waypoint_msg)

        # Use the explorer's method if available
        if hasattr(self.explorer, "get_unvisited_corners"):
            corners_remaining = len(self.explorer.get_unvisited_corners())
        else:
            corners_remaining = 0

        corners_msg = Int32()
        corners_msg.data = corners_remaining
        self.corners_remaining_pub.publish(corners_msg)

        time_msg = Float64()
        time_msg.data = computation_time
        self.computation_time_pub.publish(time_msg)

        # Publish time-based exploration info
        stats = self.explorer.get_exploration_stats()

        exploration_time_msg = Float64()
        exploration_time_msg.data = stats.get("total_time", 0.0)
        self.exploration_time_pub.publish(exploration_time_msg)

        completion_msg = Float64()
        completion_msg.data = stats.get("completion_rate", 0.0)
        self.completion_rate_pub.publish(completion_msg)

        if self.exploration_start_time is not None:
            elapsed_time = (
                self.get_clock().now() - self.exploration_start_time
            ).nanoseconds / 1e9
            time_remaining = max(0, self.exploration_duration - elapsed_time)

            time_remaining_msg = Float64()
            time_remaining_msg.data = time_remaining
            self.time_remaining_pub.publish(time_remaining_msg)

    # ========================
    # CONTROL LOGIC FUNCTIONS
    # ========================

    def generate_next_waypoint(self):
        """Generate next waypoint using butterfly explorer with time constraints."""
        if not self.enable_path_generation:
            return

        start_time = self.get_clock().now()

        # Calculate time remaining for exploration
        if self.exploration_start_time is not None:
            elapsed_time = (
                self.get_clock().now() - self.exploration_start_time
            ).nanoseconds / 1e9
            time_remaining = max(0, self.exploration_duration - elapsed_time)

            if time_remaining <= 0:
                self.get_logger().warn(
                    "Exploration time limit reached. Stopping waypoint generation."
                )
                self.enable_path_generation = False
                return
        else:
            time_remaining = self.exploration_duration

        # Generate waypoint with time constraints
        waypoint = self.explorer.generate_next_waypoint(
            time_remaining, self.exploration_duration
        )
        self.current_waypoint = np.array(waypoint)
        self.waypoint_reached = False

        computation_time = (
            self.get_clock().now() - start_time
        ).nanoseconds / 1e9

        self.publish_debug_info(waypoint, computation_time)

        # Get exploration stats for logging
        stats = self.explorer.get_exploration_stats()
        unvisited_count = (
            len(self.explorer.get_unvisited_corners())
            if hasattr(self.explorer, "get_unvisited_corners")
            else 0
        )

        self.get_logger().info(
            f"New waypoint generated: [{waypoint[0]:.2f}, {waypoint[1]:.2f}, {waypoint[2]:.2f}], "
            f"Corners remaining: {unvisited_count}, "
            f"Time remaining: {time_remaining:.1f}s, "
            f"Completion: {stats.get('completion_rate', 0):.1%}"
        )

    def auto_arm_and_offboard(self):
        """Automatically arm the vehicle and set offboard mode."""
        if not self.px4_available:
            return

        self.offboard_setpoint_counter += 1

        if self.offboard_setpoint_counter >= 10:
            if self.arming_state != 2:
                self.arm_vehicle()
            elif self.vehicle_armed and not self.offboard_mode_active:
                self.set_offboard_mode()

    def arm_vehicle(self):
        """Send arm command to the vehicle."""
        if not self.px4_available or VehicleCommand is None:
            return

        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = 1.0
        msg.param2 = 0.0
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.command = 400  # VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        if self.vehicle_command_pub is not None:
            self.vehicle_command_pub.publish(msg)
            self.get_logger().info("Arm command sent")

    def set_offboard_mode(self):
        """Send command to switch to offboard mode."""
        if not self.px4_available or VehicleCommand is None:
            return

        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = 1.0
        msg.param2 = 6.0  # OFFBOARD mode
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.command = 176  # VEHICLE_CMD_DO_SET_MODE
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        if self.vehicle_command_pub is not None:
            self.vehicle_command_pub.publish(msg)
            self.get_logger().info("Offboard mode command sent")

    def reset_exploration(self):
        """Reset the exploration to start over."""
        self._initialize_explorer()
        self.current_waypoint = None
        self.waypoint_reached = True
        self.get_logger().info("Exploration reset")

    # ========================
    # UTILITY FUNCTIONS
    # ========================

    def get_exploration_stats(self):
        """Get current exploration statistics."""
        stats = self.explorer.get_exploration_stats()

        # Add ROS-specific timing information
        if self.exploration_start_time is not None:
            elapsed_time = (
                self.get_clock().now() - self.exploration_start_time
            ).nanoseconds / 1e9
            stats["ros_elapsed_time"] = elapsed_time
            stats["ros_time_remaining"] = max(
                0, self.exploration_duration - elapsed_time
            )

        return stats

    def get_vehicle_status(self):
        """Get current vehicle status for debugging."""
        return {
            "armed": self.vehicle_armed,
            "offboard_active": self.offboard_mode_active,
            "nav_state": self.nav_state,
            "arming_state": self.arming_state,
            "setpoint_counter": self.offboard_setpoint_counter,
            "px4_available": self.px4_available,
        }


def main(args=None):
    """Main function to run the bioinspired path generator node."""
    rclpy.init(args=args)

    node = None
    try:
        node = BioinspiredPathGenerator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            stats = node.get_exploration_stats()
            if stats:
                node.get_logger().info("=== Final Exploration Statistics ===")
                node.get_logger().info(
                    f"Corners visited: {stats.get('corners_visited', 0)}/{node.explorer.total_corners}"
                )
                node.get_logger().info(
                    f"Completion rate: {stats.get('completion_rate', 0):.1%}"
                )
                node.get_logger().info(
                    f"Total steps: {stats.get('total_steps', 0)}"
                )

            node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
