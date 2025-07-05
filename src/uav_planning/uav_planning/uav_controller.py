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
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
import time

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

from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


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

        # Callback group for concurrent processing
        self.callback_group = ReentrantCallbackGroup()

        # QoS profile for PX4 compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Initialize state variables
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.current_target = None
        self.vehicle_position_valid = False
        
        # UAV state variables
        self.vehicle_armed = False
        self.offboard_mode_active = False
        self.nav_state = 0
        self.arming_state = 0
        self.offboard_setpoint_counter = 0
        
        # Control parameters
        self.trajectory_publish_rate = 20.0  # Hz
        self.control_enabled = False
        
        # State tracking for logging
        self._last_arming_state = None
        self._last_nav_state = None

        if self.px4_available:
            # PX4 Publishers
            self.trajectory_setpoint_pub = self.create_publisher(
                TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile
            )

            self.offboard_control_mode_pub = self.create_publisher(
                OffboardControlMode,
                "/fmu/in/offboard_control_mode",
                qos_profile,
            )

            self.vehicle_command_pub = self.create_publisher(
                VehicleCommand,
                "/fmu/in/vehicle_command",
                qos_profile,
            )

            # PX4 Subscribers
            self.vehicle_odometry_sub = self.create_subscription(
                VehicleOdometry,
                "/fmu/out/vehicle_odometry",
                self.vehicle_odometry_callback,
                qos_profile,
                callback_group=self.callback_group,
            )

            self.vehicle_status_sub = self.create_subscription(
                VehicleStatus,
                "/fmu/out/vehicle_status",
                self.vehicle_status_callback,
                qos_profile,
                callback_group=self.callback_group,
            )

        # ROS 2 standard interfaces
        # Subscriber for waypoint commands
        self.waypoint_sub = self.create_subscription(
            PoseStamped,
            "/uav/waypoint",
            self.waypoint_callback,
            10,
            callback_group=self.callback_group,
        )

        # Publisher for current position
        self.position_pub = self.create_publisher(
            PoseStamped,
            "/uav/current_pose",
            10
        )

        # Publisher for current velocity
        self.velocity_pub = self.create_publisher(
            TwistStamped,
            "/uav/current_velocity",
            10
        )

        # Publisher for vehicle status
        self.status_pub = self.create_publisher(
            Bool,
            "/uav/armed",
            10
        )

        # Services
        self.arm_service = self.create_service(
            SetBool,
            "/uav/arm",
            self.arm_service_callback,
            callback_group=self.callback_group,
        )

        self.offboard_service = self.create_service(
            SetBool,
            "/uav/set_offboard_mode",
            self.offboard_service_callback,
            callback_group=self.callback_group,
        )

        # Control timers
        self.trajectory_timer = self.create_timer(
            1.0 / self.trajectory_publish_rate,
            self.publish_trajectory_callback,
            callback_group=self.callback_group
        )

        self.offboard_timer = self.create_timer(
            0.1,  # 10 Hz
            self.publish_offboard_control_mode,
            callback_group=self.callback_group
        )

        self.status_timer = self.create_timer(
            0.5,  # 2 Hz
            self.publish_status_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info("UAV Controller initialized")

    def vehicle_odometry_callback(self, msg):
        """Update current vehicle position and velocity from odometry."""
        # Convert from PX4 NED to ROS ENU coordinate frame
        self.current_position = np.array([
            msg.position[1],    # East
            msg.position[0],    # North  
            -msg.position[2],   # Up (PX4 uses NED, convert to positive up)
        ])
        
        self.current_velocity = np.array([
            msg.velocity[1],    # East velocity
            msg.velocity[0],    # North velocity
            -msg.velocity[2],   # Up velocity
        ])
        
        self.vehicle_position_valid = True

    def vehicle_status_callback(self, msg):
        """Update vehicle status information."""
        self.vehicle_armed = msg.arming_state == 2  # ARMING_STATE_ARMED
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.offboard_mode_active = msg.nav_state == 14  # NAV_STATE_OFFBOARD

        # Log state changes
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

    def waypoint_callback(self, msg):
        """Receive new waypoint target."""
        self.current_target = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        
        self.get_logger().info(
            f"New waypoint received: [{self.current_target[0]:.2f}, "
            f"{self.current_target[1]:.2f}, {self.current_target[2]:.2f}]"
        )

    def publish_trajectory_callback(self):
        """Publish trajectory setpoint for PX4."""
        if (
            self.current_target is None
            or not self.px4_available
            or TrajectorySetpoint is None
        ):
            return

        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # Convert from ROS ENU to PX4 NED coordinate frame
        msg.position[0] = float(self.current_target[1])  # North
        msg.position[1] = float(self.current_target[0])  # East
        msg.position[2] = -float(self.current_target[2]) # Down

        # Let PX4 handle velocity and acceleration
        msg.velocity[0] = float("nan")
        msg.velocity[1] = float("nan")
        msg.velocity[2] = float("nan")

        # Calculate yaw to face movement direction
        if self.vehicle_position_valid and self.current_target is not None:
            direction = self.current_target - self.current_position
            if np.linalg.norm(direction[:2]) > 0.1:
                msg.yaw = float(np.arctan2(direction[1], direction[0]))
            else:
                msg.yaw = float("nan")
        else:
            msg.yaw = float("nan")

        self.trajectory_setpoint_pub.publish(msg)

    def publish_offboard_control_mode(self):
        """Publish offboard control mode for PX4."""
        if not self.px4_available or OffboardControlMode is None:
            return

        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_control_mode_pub.publish(msg)
        self.offboard_setpoint_counter += 1

    def publish_status_callback(self):
        """Publish current vehicle status and position."""
        # Publish current position
        if self.vehicle_position_valid:
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

    def arm_service_callback(self, request, response):
        """Service callback for arming/disarming."""
        if request.data:
            success = self.arm_vehicle()
            response.message = "Arm command sent" if success else "Failed to send arm command"
        else:
            success = self.disarm_vehicle()
            response.message = "Disarm command sent" if success else "Failed to send disarm command"
        
        response.success = success
        return response

    def offboard_service_callback(self, request, response):
        """Service callback for offboard mode."""
        if request.data:
            success = self.set_offboard_mode()
            response.message = "Offboard mode command sent" if success else "Failed to send offboard command"
        else:
            success = self.set_manual_mode()
            response.message = "Manual mode command sent" if success else "Failed to send manual mode command"
        
        response.success = success
        return response

    def arm_vehicle(self):
        """Send arm command to the vehicle."""
        if not self.px4_available or VehicleCommand is None:
            return False

        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = 1.0  # Arm
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

        self.vehicle_command_pub.publish(msg)
        self.get_logger().info("Arm command sent")
        return True

    def disarm_vehicle(self):
        """Send disarm command to the vehicle."""
        if not self.px4_available or VehicleCommand is None:
            return False

        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = 0.0  # Disarm
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

        self.vehicle_command_pub.publish(msg)
        self.get_logger().info("Disarm command sent")
        return True

    def set_offboard_mode(self):
        """Send command to switch to offboard mode."""
        if not self.px4_available or VehicleCommand is None:
            return False

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

        self.vehicle_command_pub.publish(msg)
        self.get_logger().info("Offboard mode command sent")
        return True

    def set_manual_mode(self):
        """Send command to switch to manual mode."""
        if not self.px4_available or VehicleCommand is None:
            return False

        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = 1.0
        msg.param2 = 1.0  # MANUAL mode
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

        self.vehicle_command_pub.publish(msg)
        self.get_logger().info("Manual mode command sent")
        return True


def main(args=None):
    """Main function to run the UAV controller."""
    rclpy.init(args=args)

    node = UAVController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
