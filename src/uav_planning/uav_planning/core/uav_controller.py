#!/usr/bin/env python3
"""UAV Controller ROS 2 node for continuous vehicle control and monitoring."""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
import numpy as np

from px4_msgs.msg import (
    TrajectorySetpoint,
    VehicleOdometry,
    OffboardControlMode,
    VehicleCommand,
    VehicleStatus,
)

from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

try:
    from uav_interfaces.msg import UAVState
except ImportError:
    UAVState = None


class UAVController(Node):
    """
    Continuous UAV controller that handles:
    - Vehicle arming/disarming
    - Mode switching (offboard/manual)
    - Trajectory following
    - Position/velocity control
    - Vehicle status monitoring
    """

    def __init__(self):
        super().__init__("uav_controller")

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Create QoS profile for internal UAV topics
        internal_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # State variables
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.current_target = None
        self.vehicle_position_valid = False

        self.vehicle_armed = False
        self.offboard_mode_active = False
        self.nav_state = 0
        self.arming_state = 0
        self.offboard_setpoint_counter = 0

        self.current_uav_state = 0
        self.state_machine_available = UAVState is not None

        self.trajectory_publish_rate = 20.0
        self.control_enabled = False

        self._last_arming_state = None
        self._last_nav_state = None

        # Startup sequence tracking
        self.startup_counter = 0
        self.min_setpoints_before_offboard = 10

        # Logging rate limiters
        self.trajectory_log_counter = 0
        self.offboard_log_counter = 0
        self.status_log_counter = 0
        self.odometry_log_counter = 0

        # Publishers
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            10,
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            10,
        )

        # Subscribers
        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.vehicle_odometry_callback,
            px4_qos,
        )

        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.vehicle_status_callback,
            px4_qos,
        )

        self.waypoint_sub = self.create_subscription(
            PoseStamped,
            "/uav/waypoint",
            self.waypoint_callback,
            internal_qos,
        )

        if self.state_machine_available:
            self.state_sub = self.create_subscription(
                UAVState,
                "/uav/state",
                self.state_callback,
                internal_qos,
            )

        # Status publishers
        self.position_pub = self.create_publisher(
            PoseStamped, "/uav/current_pose", internal_qos
        )

        self.velocity_pub = self.create_publisher(
            TwistStamped, "/uav/current_velocity", internal_qos
        )

        self.status_pub = self.create_publisher(
            Bool, "/uav/armed", internal_qos
        )

        # Services
        self.arm_service = self.create_service(
            SetBool,
            "/uav/arm",
            self.arm_service_callback,
        )

        self.offboard_service = self.create_service(
            SetBool,
            "/uav/set_offboard_mode",
            self.offboard_service_callback,
        )

        # Timers - No callback groups used
        self.trajectory_timer = self.create_timer(
            1.0 / self.trajectory_publish_rate,
            self.publish_trajectory_callback,
        )

        self.offboard_timer = self.create_timer(
            0.1,
            self.publish_offboard_control_mode,
        )

        self.status_timer = self.create_timer(
            0.5,
            self.publish_status_callback,
        )

        self.get_logger().info("UAV Controller initialized")
        self.get_logger().info(
            f"Timer frequencies: Trajectory={self.trajectory_publish_rate}Hz, Offboard=10Hz, Status=2Hz"
        )

    def vehicle_odometry_callback(self, msg):
        """Update current vehicle position and velocity from odometry."""
        # Convert NED -> ENU
        self.current_position = np.array(
            [
                msg.position[1],  # North -> East
                msg.position[0],  # East -> North
                -msg.position[2],  # Down -> Up
            ]
        )

        self.current_velocity = np.array(
            [
                msg.velocity[1],  # North -> East
                msg.velocity[0],  # East -> North
                -msg.velocity[2],  # Down -> Up
            ]
        )

        self.vehicle_position_valid = True

        # Rate-limited logging (assuming ~50Hz odometry)
        self.odometry_log_counter += 1
        if self.odometry_log_counter % 50 == 0:  # Log every second
            self.get_logger().info(
                f"Position updated: [{self.current_position[0]:.2f}, "
                f"{self.current_position[1]:.2f}, {self.current_position[2]:.2f}]"
            )

    def vehicle_status_callback(self, msg):
        """Update vehicle status information."""
        self.vehicle_armed = msg.arming_state == 2
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.offboard_mode_active = msg.nav_state == 14

        # Log arming state changes
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

        # Log navigation state changes
        if (
            hasattr(self, "_last_nav_state")
            and self._last_nav_state != msg.nav_state
        ):
            nav_states = {
                14: "OFFBOARD",
                1: "MANUAL",
                3: "ALTCTL",
                4: "POSCTL",
            }
            self.get_logger().info(
                f"Nav state changed: {nav_states.get(msg.nav_state, f'NAV_STATE({msg.nav_state})')}"
            )
        self._last_nav_state = msg.nav_state

    def waypoint_callback(self, msg):
        """Receive new waypoint target."""
        self.get_logger().info(
            f"Received new waypoint: [{msg.pose.position.x:.2f}, "
            f"{msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}]"
        )
        self.current_target = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        )

    def publish_trajectory_callback(self):
        """
        Publish trajectory setpoint for PX4.
        CRITICAL: This must ALWAYS publish a setpoint, even without a target.
        """
        self.trajectory_log_counter += 1

        # Log every second (20Hz -> every 20 calls)
        if self.trajectory_log_counter % 20 == 0:
            self.get_logger().info(
                f"Trajectory callback executing (count: {self.trajectory_log_counter})"
            )

        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        if self.current_target is None:
            # Default hover position at 2m altitude
            msg.position[0] = 0.0  # North
            msg.position[1] = 0.0  # East
            msg.position[2] = -2.0  # Down (2m up in ENU)

            # Log state changes or every 5 seconds
            if self.trajectory_log_counter % 100 == 0:
                self.get_logger().info(
                    "Publishing default hover setpoint at 2m altitude"
                )
        else:
            msg.position[0] = float(self.current_target[1])  # East -> North
            msg.position[1] = float(self.current_target[0])  # North -> East
            msg.position[2] = -float(self.current_target[2])  # Up -> Down

            # Calculate yaw to face target direction
            if self.vehicle_position_valid:
                direction = self.current_target - self.current_position
                if np.linalg.norm(direction[:2]) > 0.1:
                    msg.yaw = float(np.arctan2(direction[1], direction[0]))
                else:
                    msg.yaw = float("nan")
            else:
                msg.yaw = float("nan")

            # Log every 2 seconds when tracking target
            if self.trajectory_log_counter % 40 == 0:
                self.get_logger().info(
                    f"Publishing target setpoint: [{self.current_target[0]:.2f}, "
                    f"{self.current_target[1]:.2f}, {self.current_target[2]:.2f}]"
                )

        # Always publish the message
        self.trajectory_setpoint_pub.publish(msg)
        self.startup_counter += 1

        # Log first few setpoints to confirm publishing
        if self.startup_counter <= 5:
            self.get_logger().info(
                f"Trajectory setpoint #{self.startup_counter} published"
            )

    def publish_offboard_control_mode(self):
        """
        Publish offboard control mode heartbeat for PX4.
        This signal is sent periodically, even if the UAV is disarmed.
        """
        self.offboard_log_counter += 1

        # Log every second (10Hz -> every 10 calls)
        if self.offboard_log_counter % 10 == 0:
            self.get_logger().info(
                f"Offboard control mode callback executing (count: {self.offboard_log_counter})"
            )

        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_control_mode_pub.publish(msg)
        self.offboard_setpoint_counter += 1

        # Log first few offboard signals
        if self.offboard_setpoint_counter <= 5:
            self.get_logger().info(
                f"Offboard control mode #{self.offboard_setpoint_counter} published"
            )

    def publish_status_callback(self):
        """Publish current vehicle status and position."""
        self.status_log_counter += 1

        # Log every 2 seconds (2Hz -> every 4 calls)
        if self.status_log_counter % 4 == 0:
            self.get_logger().info(
                f"Status callback executing (count: {self.status_log_counter})"
            )

        if self.vehicle_position_valid:
            # Publish current pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = float(self.current_position[0])
            pose_msg.pose.position.y = float(self.current_position[1])
            pose_msg.pose.position.z = float(self.current_position[2])
            self.position_pub.publish(pose_msg)

            # Publish current velocity
            velocity_msg = TwistStamped()
            velocity_msg.header.stamp = self.get_clock().now().to_msg()
            velocity_msg.header.frame_id = "map"
            velocity_msg.twist.linear.x = float(self.current_velocity[0])
            velocity_msg.twist.linear.y = float(self.current_velocity[1])
            velocity_msg.twist.linear.z = float(self.current_velocity[2])
            self.velocity_pub.publish(velocity_msg)

        # Publish armed status
        status_msg = Bool()
        status_msg.data = self.vehicle_armed
        self.status_pub.publish(status_msg)

        # Log status periodically
        if hasattr(self, "_status_count"):
            self._status_count += 1
        else:
            self._status_count = 1

        if self._status_count % 10 == 0:  # Every 5 seconds
            self.get_logger().info(
                f"Status: Armed={self.vehicle_armed}, Offboard={self.offboard_mode_active}, "
                f"Position_valid={self.vehicle_position_valid}, Setpoints={self.offboard_setpoint_counter}"
            )

    def arm_service_callback(self, request, response):
        """Service callback for arming/disarming with retry until confirmed."""
        import time

        if request.data:
            # Check prerequisites for arming
            if (
                self.offboard_setpoint_counter
                < self.min_setpoints_before_offboard
            ):
                response.success = False
                response.message = f"Cannot arm: Need at least {self.min_setpoints_before_offboard} setpoints published (current: {self.offboard_setpoint_counter})"
                return response

            # Retry arming until confirmed
            max_attempts = 10
            attempt_delay = 0.5  # seconds

            self.get_logger().info("Attempting to arm vehicle...")

            for attempt in range(max_attempts):
                if self.vehicle_armed:
                    response.success = True
                    response.message = (
                        f"Vehicle armed successfully after {attempt} attempts"
                    )
                    self.get_logger().info(response.message)
                    return response

                self.arm_vehicle()
                self.get_logger().info(
                    f"Arm attempt {attempt + 1}/{max_attempts}"
                )

                # Wait for status update
                time.sleep(attempt_delay)

            response.success = False
            response.message = f"Failed to arm after {max_attempts} attempts"
            self.get_logger().error(response.message)

        else:
            # Retry disarming until confirmed
            max_attempts = 10
            attempt_delay = 0.5  # seconds

            self.get_logger().info("Attempting to disarm vehicle...")

            for attempt in range(max_attempts):
                if not self.vehicle_armed:
                    response.success = True
                    response.message = f"Vehicle disarmed successfully after {attempt} attempts"
                    self.get_logger().info(response.message)
                    return response

                self.disarm_vehicle()
                self.get_logger().info(
                    f"Disarm attempt {attempt + 1}/{max_attempts}"
                )

                # Wait for status update
                time.sleep(attempt_delay)

            response.success = False
            response.message = f"Failed to disarm after {max_attempts} attempts"
            self.get_logger().error(response.message)

        return response

    def offboard_service_callback(self, request, response):
        """Service callback for offboard mode with retry until confirmed."""
        import time

        if request.data:
            # Check prerequisites for offboard mode
            if not self.vehicle_armed:
                response.success = False
                response.message = (
                    "Cannot enter offboard mode: vehicle not armed"
                )
                return response

            if not self.vehicle_position_valid:
                response.success = False
                response.message = (
                    "Cannot enter offboard mode: no valid position data"
                )
                return response

            if (
                self.offboard_setpoint_counter
                < self.min_setpoints_before_offboard
            ):
                response.success = False
                response.message = f"Cannot enter offboard mode: need at least {self.min_setpoints_before_offboard} setpoints (current: {self.offboard_setpoint_counter})"
                return response

            # Retry offboard mode until confirmed
            max_attempts = 10
            attempt_delay = 0.5  # seconds

            self.get_logger().info("Attempting to enter offboard mode...")

            for attempt in range(max_attempts):
                if self.offboard_mode_active:
                    response.success = True
                    response.message = f"Offboard mode activated successfully after {attempt} attempts"
                    self.get_logger().info(response.message)
                    return response

                self.set_offboard_mode()
                self.get_logger().info(
                    f"Offboard mode attempt {attempt + 1}/{max_attempts}"
                )

                # Wait for status update
                time.sleep(attempt_delay)

            response.success = False
            response.message = (
                f"Failed to enter offboard mode after {max_attempts} attempts"
            )
            self.get_logger().error(response.message)

        else:
            # Retry manual mode until confirmed (not offboard)
            max_attempts = 10
            attempt_delay = 0.5  # seconds

            self.get_logger().info("Attempting to enter manual mode...")

            for attempt in range(max_attempts):
                if not self.offboard_mode_active:
                    response.success = True
                    response.message = f"Manual mode activated successfully after {attempt} attempts"
                    self.get_logger().info(response.message)
                    return response

                self.set_manual_mode()
                self.get_logger().info(
                    f"Manual mode attempt {attempt + 1}/{max_attempts}"
                )

                # Wait for status update
                time.sleep(attempt_delay)

            response.success = False
            response.message = (
                f"Failed to exit offboard mode after {max_attempts} attempts"
            )
            self.get_logger().error(response.message)

        return response

    def arm_vehicle(self):
        """Send arm command to the vehicle."""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = 1.0  # Arm
        msg.param2 = 0.0
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.vehicle_command_pub.publish(msg)
        self.get_logger().info("Arm command sent")
        return True

    def disarm_vehicle(self):
        """Send disarm command to the vehicle."""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = 0.0  # Disarm
        msg.param2 = 0.0
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.vehicle_command_pub.publish(msg)
        self.get_logger().info("Disarm command sent")
        return True

    def set_offboard_mode(self):
        """Send command to switch to offboard mode."""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = 1.0  # Enable
        msg.param2 = 6.0  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.command = 176  # MAV_CMD_DO_SET_MODE
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.vehicle_command_pub.publish(msg)
        self.get_logger().info("Offboard mode command sent")
        return True

    def set_manual_mode(self):
        """Send command to switch to manual mode."""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = 1.0  # Enable
        msg.param2 = 1.0  # PX4_CUSTOM_MAIN_MODE_MANUAL
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.command = 176  # MAV_CMD_DO_SET_MODE
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.vehicle_command_pub.publish(msg)
        self.get_logger().info("Manual mode command sent")
        return True

    def state_callback(self, msg):
        """Update current UAV state from state machine."""
        if not self.state_machine_available:
            return

        old_state = self.current_uav_state
        self.current_uav_state = msg.state

        if old_state != self.current_uav_state:
            self.get_logger().info(
                f"UAV state changed: {msg.state_description}"
            )

        # Update control enabled based on state
        if msg.state == 0:
            self.control_enabled = False
        elif msg.state in [1, 2, 3, 4, 5]:
            self.control_enabled = True


def main(args=None):
    """Main function to run the UAV controller."""
    rclpy.init(args=args)

    node = UAVController()

    try:
        node.get_logger().info("Starting UAV Controller - spinning node...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        node.get_logger().info("Shutting down UAV Controller")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
