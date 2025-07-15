#!/usr/bin/env python3
"""UAV State Machine ROS 2 node for coordinating UAV operations."""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
import numpy as np

from geometry_msgs.msg import PoseStamped, Point
from std_srvs.srv import SetBool

try:
    from uav_interfaces.msg import UAVState
    from uav_interfaces.action import PathGeneration
except ImportError:
    UAVState = None
    PathGeneration = None

try:
    from px4_msgs.msg import VehicleStatus, VehicleOdometry
except ImportError:
    VehicleStatus = None
    VehicleOdometry = None


class UAVStateMachine(Node):
    """
    UAV State Machine that coordinates all UAV operations:
    - State transitions (IDLE -> TAKEOFF -> ACTION_IN_PROGRESS -> HOVER -> LANDING)
    - Emergency handling
    - Action coordination with path generator
    - Controller coordination
    """

    def __init__(self):
        super().__init__("uav_state_machine")

        # Check if UAV interfaces are available
        if UAVState is None or PathGeneration is None:
            self.get_logger().error(
                "uav_interfaces not available. Please build the uav_interfaces package."
            )
            return

        # QoS profile for subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        internal_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # State constants from UAVState.msg
        self.IDLE = UAVState.IDLE
        self.TAKEOFF = UAVState.TAKEOFF
        self.ACTION_IN_PROGRESS = UAVState.ACTION_IN_PROGRESS
        self.HOVER = UAVState.HOVER
        self.LANDING = UAVState.LANDING
        self.EMERGENCY = UAVState.EMERGENCY

        # Current state
        self.current_state = self.IDLE
        self.state_start_time = self.get_clock().now()
        self.target_position = Point()

        # Callback group for concurrent processing
        self.callback_group = ReentrantCallbackGroup()

        # State variables
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_armed = False
        self.offboard_mode_active = False
        self.nav_state = 0
        self.arming_state = 0
        self.vehicle_position_valid = False

        # Action variables
        self.current_action_client = None
        self.current_goal_handle = None
        self.action_in_progress = False

        # Publishers
        self.state_pub = self.create_publisher(
            UAVState, "/uav/state", internal_qos
        )

        self.waypoint_pub = self.create_publisher(
            PoseStamped, "/uav/waypoint", internal_qos
        )

        # Subscribers
        if VehicleStatus is not None:
            self.vehicle_status_sub = self.create_subscription(
                VehicleStatus,
                "/fmu/out/vehicle_status",
                self.vehicle_status_callback,
                qos_profile,
                callback_group=self.callback_group,
            )

        if VehicleOdometry is not None:
            self.vehicle_odometry_sub = self.create_subscription(
                VehicleOdometry,
                "/fmu/out/vehicle_odometry",
                self.vehicle_odometry_callback,
                qos_profile,
                callback_group=self.callback_group,
            )

        # Action clients
        self.path_generation_client = ActionClient(
            self,
            PathGeneration,
            "generate_path",
            callback_group=self.callback_group,
        )

        # Service clients
        self.arm_client = self.create_client(
            SetBool, "/uav/arm", callback_group=self.callback_group
        )

        self.offboard_client = self.create_client(
            SetBool,
            "/uav/set_offboard_mode",
            callback_group=self.callback_group,
        )

        # Services
        self.start_mission_service = self.create_service(
            SetBool,
            "/uav/start_mission",
            self.start_mission_callback,
            callback_group=self.callback_group,
        )

        self.emergency_service = self.create_service(
            SetBool,
            "/uav/emergency",
            self.emergency_callback,
            callback_group=self.callback_group,
        )

        self.land_service = self.create_service(
            SetBool,
            "/uav/land",
            self.land_callback,
            callback_group=self.callback_group,
        )

        # State machine timer
        self.state_timer = self.create_timer(
            0.1,  # 10 Hz
            self.state_machine_callback,
            callback_group=self.callback_group,
        )

        # State publishing timer
        self.state_publish_timer = self.create_timer(
            0.5,  # 2 Hz
            self.publish_state_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info("UAV State Machine initialized")

    def vehicle_status_callback(self, msg):
        """Update vehicle status information."""
        self.vehicle_armed = msg.arming_state == 2  # ARMING_STATE_ARMED
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.offboard_mode_active = msg.nav_state == 14  # NAV_STATE_OFFBOARD

    def vehicle_odometry_callback(self, msg):
        """Update current vehicle position from odometry."""
        # Convert from PX4 NED to ROS ENU coordinate frame
        self.current_position = np.array(
            [
                msg.position[1],  # East -> X
                msg.position[0],  # North -> Y
                -msg.position[2],  # Down -> Z (inverted)
            ]
        )
        self.vehicle_position_valid = True

    def state_machine_callback(self):
        """Main state machine logic."""
        if self.current_state == self.IDLE:
            self._handle_idle_state()
        elif self.current_state == self.TAKEOFF:
            self._handle_takeoff_state()
        elif self.current_state == self.ACTION_IN_PROGRESS:
            self._handle_action_in_progress_state()
        elif self.current_state == self.HOVER:
            self._handle_hover_state()
        elif self.current_state == self.LANDING:
            self._handle_landing_state()
        elif self.current_state == self.EMERGENCY:
            self._handle_emergency_state()

    def _handle_idle_state(self):
        """Handle IDLE state logic."""
        # Wait for mission start request
        pass

    def _handle_takeoff_state(self):
        """Handle TAKEOFF state logic."""
        if not self.vehicle_armed:
            # Try to arm the vehicle
            self._arm_vehicle()
            return

        if not self.offboard_mode_active:
            # Try to set offboard mode
            self._set_offboard_mode()
            return

        # Check if takeoff is complete (reached target altitude)
        if (
            self.vehicle_position_valid
            and self.current_position[2] >= self.target_position.z - 0.5
        ):
            self.get_logger().info("Takeoff complete, transitioning to HOVER")
            self._transition_to_state(self.HOVER, "Takeoff completed")

    def _handle_action_in_progress_state(self):
        """Handle ACTION_IN_PROGRESS state logic."""
        # Check if action is still running
        if not self.action_in_progress:
            self.get_logger().info("Action completed, transitioning to HOVER")
            self._transition_to_state(self.HOVER, "Action completed")

        # Monitor for emergency conditions
        if self._check_emergency_conditions():
            self._handle_emergency()

    def _handle_hover_state(self):
        """Handle HOVER state logic."""
        # Maintain current position
        if self.vehicle_position_valid:
            self._send_waypoint(self.current_position)

        # Monitor for emergency conditions
        if self._check_emergency_conditions():
            self._handle_emergency()

    def _handle_landing_state(self):
        """Handle LANDING state logic."""
        # Check if landing is complete
        if (
            self.vehicle_position_valid
            and self.current_position[2] <= 0.2
            and not self.vehicle_armed
        ):
            self.get_logger().info("Landing complete, transitioning to IDLE")
            self._transition_to_state(self.IDLE, "Landing completed")

    def _handle_emergency_state(self):
        """Handle EMERGENCY state logic."""
        # Cancel any ongoing actions
        if self.action_in_progress and self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
            self.action_in_progress = False

        # Hover at current position
        if self.vehicle_position_valid:
            self._send_waypoint(self.current_position)

    def _transition_to_state(self, new_state, description=""):
        """Transition to a new state."""
        old_state_name = self._get_state_name(self.current_state)
        new_state_name = self._get_state_name(new_state)

        self.get_logger().info(
            f"State transition: {old_state_name} -> {new_state_name}: {description}"
        )

        self.current_state = new_state
        self.state_start_time = self.get_clock().now()

    def _get_state_name(self, state):
        """Get human-readable state name."""
        state_names = {
            self.IDLE: "IDLE",
            self.TAKEOFF: "TAKEOFF",
            self.ACTION_IN_PROGRESS: "ACTION_IN_PROGRESS",
            self.HOVER: "HOVER",
            self.LANDING: "LANDING",
            self.EMERGENCY: "EMERGENCY",
        }
        return state_names.get(state, "UNKNOWN")

    def _check_emergency_conditions(self):
        """Check for emergency conditions."""
        # Check if vehicle is disarmed unexpectedly
        if not self.vehicle_armed and self.current_state != self.IDLE:
            return True

        # Check if offboard mode is lost unexpectedly
        if not self.offboard_mode_active and self.current_state in [
            self.ACTION_IN_PROGRESS,
            self.HOVER,
        ]:
            return True

        return False

    def _handle_emergency(self):
        """Handle emergency condition."""
        self.get_logger().error("Emergency condition detected!")
        self._transition_to_state(self.EMERGENCY, "Emergency detected")

    def _send_waypoint(self, position):
        """Send waypoint to controller."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])

        self.waypoint_pub.publish(msg)

    def _arm_vehicle(self):
        """Request vehicle arming."""
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Arm service not available")
            return

        request = SetBool.Request()
        request.data = True

        future = self.arm_client.call_async(request)
        future.add_done_callback(self._arm_response_callback)

    def _arm_response_callback(self, future):
        """Handle arm service response."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Vehicle arming requested")
            else:
                self.get_logger().warn(
                    f"Failed to arm vehicle: {response.message}"
                )
        except Exception as e:
            self.get_logger().error(f"Arm service call failed: {e}")

    def _set_offboard_mode(self):
        """Request offboard mode."""
        if not self.offboard_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Offboard service not available")
            return

        request = SetBool.Request()
        request.data = True

        future = self.offboard_client.call_async(request)
        future.add_done_callback(self._offboard_response_callback)

    def _offboard_response_callback(self, future):
        """Handle offboard service response."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Offboard mode requested")
            else:
                self.get_logger().warn(
                    f"Failed to set offboard mode: {response.message}"
                )
        except Exception as e:
            self.get_logger().error(f"Offboard service call failed: {e}")

    def start_mission_callback(self, request, response):
        """Handle start mission service request."""
        if request.data:
            if self.current_state == self.IDLE:
                # Set takeoff target position
                self.target_position.x = self.current_position[0]
                self.target_position.y = self.current_position[1]
                self.target_position.z = 5.0  # Default takeoff altitude

                self._transition_to_state(
                    self.TAKEOFF, "Mission start requested"
                )

                # Start path generation action after takeoff
                self._start_path_generation_action()

                response.success = True
                response.message = "Mission started"
            else:
                response.success = False
                response.message = f"Cannot start mission from state {self._get_state_name(self.current_state)}"
        else:
            # Stop mission
            if self.current_state in [self.ACTION_IN_PROGRESS, self.HOVER]:
                self._transition_to_state(
                    self.LANDING, "Mission stop requested"
                )
                response.success = True
                response.message = "Mission stopped, landing initiated"
            else:
                response.success = False
                response.message = f"Cannot stop mission from state {self._get_state_name(self.current_state)}"

        return response

    def emergency_callback(self, request, response):
        """Handle emergency service request."""
        if request.data:
            self._handle_emergency()
            response.success = True
            response.message = "Emergency mode activated"
        else:
            if self.current_state == self.EMERGENCY:
                self._transition_to_state(self.HOVER, "Emergency cleared")
                response.success = True
                response.message = "Emergency cleared, returning to hover"
            else:
                response.success = False
                response.message = "Not in emergency state"

        return response

    def land_callback(self, request, response):
        """Handle land service request."""
        if request.data:
            if self.current_state in [
                self.HOVER,
                self.ACTION_IN_PROGRESS,
                self.EMERGENCY,
            ]:
                # Cancel any ongoing actions
                if self.action_in_progress and self.current_goal_handle:
                    self.current_goal_handle.cancel_goal_async()
                    self.action_in_progress = False

                self._transition_to_state(self.LANDING, "Landing requested")
                response.success = True
                response.message = "Landing initiated"
            else:
                response.success = False
                response.message = f"Cannot land from state {self._get_state_name(self.current_state)}"
        else:
            response.success = False
            response.message = "Invalid request"

        return response

    def _start_path_generation_action(self):
        """Start the path generation action."""
        if PathGeneration is None:
            self.get_logger().error("PathGeneration action not available")
            return

        if not self.path_generation_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "Path generation action server not available"
            )
            return

        # Create action goal
        goal_msg = PathGeneration.Goal()
        goal_msg.x_min = -50.0
        goal_msg.x_max = 50.0
        goal_msg.y_min = -50.0
        goal_msg.y_max = 50.0
        goal_msg.z_min = 3.0
        goal_msg.z_max = 10.0
        goal_msg.alpha = 1.5
        goal_msg.visit_threshold = 2.0

        self.get_logger().info("Sending path generation goal...")

        send_goal_future = self.path_generation_client.send_goal_async(
            goal_msg, feedback_callback=self._path_generation_feedback_callback
        )
        send_goal_future.add_done_callback(
            self._path_generation_goal_response_callback
        )

    def _path_generation_goal_response_callback(self, future):
        """Handle path generation goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Path generation goal rejected")
            self._transition_to_state(
                self.HOVER, "Path generation goal rejected"
            )
            return

        self.get_logger().info("Path generation goal accepted")
        self.current_goal_handle = goal_handle
        self.action_in_progress = True
        self._transition_to_state(
            self.ACTION_IN_PROGRESS, "Path generation started"
        )

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            self._path_generation_result_callback
        )

    def _path_generation_result_callback(self, future):
        """Handle path generation result."""
        result = future.result().result
        self.action_in_progress = False
        self.current_goal_handle = None

        self.get_logger().info(
            f"Path generation completed. Waypoints generated: {result.waypoints_generated}, "
            f"Coverage: {result.coverage_percentage:.1f}%"
        )

    def _path_generation_feedback_callback(self, feedback_msg):
        """Handle path generation feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f"Path generation progress: {feedback.waypoints_generated} waypoints, "
            f"{feedback.coverage_percentage:.1f}% coverage"
        )

    def publish_state_callback(self):
        """Publish current state."""
        if UAVState is None:
            return

        msg = UAVState()
        msg.state = self.current_state
        msg.state_description = self._get_state_name(self.current_state)
        msg.state_start_time = self.state_start_time.to_msg()

        current_time = self.get_clock().now()
        duration = (current_time - self.state_start_time).nanoseconds / 1e9
        msg.state_duration = duration

        msg.target_position = self.target_position

        self.state_pub.publish(msg)


def main(args=None):
    """Main function to run the UAV state machine."""
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    node = UAVStateMachine()

    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down UAV State Machine...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
