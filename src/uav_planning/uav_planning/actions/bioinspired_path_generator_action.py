#!/usr/bin/env python3
"""Bio-inspired path generator ROS 2 action server using Levy flight patterns."""

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import asyncio

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
from uav_interfaces.action import PathGeneration
import numpy as np
import time
from ..algorithms.butterfly import ButterflyExplorer


class BioinspiredPathGeneratorActionServer(Node):
    """
    ROS 2 action server that generates bio-inspired flight paths using Levy flight patterns
    and publishes trajectory setpoints compatible with PX4 autopilot.
    """

    def __init__(self):
        super().__init__("bioinspired_path_generator_action_server")

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
                callback_group=self.callback_group,
            )

            if VehicleStatus is not None:
                self.vehicle_status_sub = self.create_subscription(
                    VehicleStatus,
                    "/fmu/out/vehicle_status_v1",
                    self.vehicle_status_callback,
                    qos_profile,
                    callback_group=self.callback_group,
                )
            else:
                self.vehicle_status_sub = None
        else:
            self.trajectory_setpoint_pub = None
            self.offboard_control_mode_pub = None
            self.vehicle_command_pub = None
            self.vehicle_odometry_sub = None
            self.vehicle_status_sub = None

        # Action server
        self._action_server = ActionServer(
            self,
            PathGeneration,
            'generate_path',
            self.execute_callback,
            callback_group=self.callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # State variables
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_waypoint = None
        self.waypoint_reached = True
        self.vehicle_position_valid = False

        # UAV state variables
        self.vehicle_armed = False
        self.offboard_mode_active = False
        self.nav_state = 0
        self.arming_state = 0
        self.offboard_setpoint_counter = 0

        # Action execution state
        self.current_goal_handle = None
        self.explorer = None
        self.start_time = None
        self.waypoints_generated = 0
        self.trajectory_publish_rate = 10.0  # Hz
        self.visit_threshold = 1.2

        # Initialize state tracking variables
        self._last_arming_state = None
        self._last_nav_state = None

        self.get_logger().info("Bioinspired path generator action server initialized")

    def goal_callback(self, goal_request):
        """Accept or reject a goal."""
        self.get_logger().info('Received goal request')
        
        # Validate goal parameters
        if goal_request.x_min >= goal_request.x_max:
            self.get_logger().warn('Invalid X bounds: x_min >= x_max')
            return GoalResponse.REJECT
            
        if goal_request.y_min >= goal_request.y_max:
            self.get_logger().warn('Invalid Y bounds: y_min >= y_max')
            return GoalResponse.REJECT
            
        if goal_request.z_min >= goal_request.z_max:
            self.get_logger().warn('Invalid Z bounds: z_min >= z_max')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle cancellation."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the path generation action."""
        self.get_logger().info('Executing path generation...')
        
        # Store current goal handle
        self.current_goal_handle = goal_handle
        goal = goal_handle.request
        
        # Initialize explorer with goal parameters
        self.explorer = ButterflyExplorer(
            x_min=goal.x_min,
            x_max=goal.x_max,
            y_min=goal.y_min,
            y_max=goal.y_max,
            z_min=goal.z_min,
            z_max=goal.z_max,
            alpha=goal.alpha,
            visit_threshold=goal.visit_threshold,
        )
        
        self.visit_threshold = goal.visit_threshold
        self.waypoints_generated = 0
        self.start_time = time.time()
        
        # Create feedback message
        feedback_msg = PathGeneration.Feedback()
        
        # Create result message
        result = PathGeneration.Result()
        
        # Log goal parameters
        self.get_logger().info(
            f"Starting exploration with bounds: "
            f"X[{goal.x_min}, {goal.x_max}], Y[{goal.y_min}, {goal.y_max}], Z[{goal.z_min}, {goal.z_max}]"
        )
        
        # Create timers for trajectory publishing and offboard control
        trajectory_timer = self.create_timer(
            1.0 / self.trajectory_publish_rate, 
            self.publish_trajectory_callback,
            callback_group=self.callback_group
        )
        
        offboard_timer = self.create_timer(
            0.1,  # 10 Hz
            self.publish_offboard_control_mode,
            callback_group=self.callback_group
        )
        
        try:
            # Automatic arming if enabled
            if goal.enable_auto_arm and self.px4_available:
                await self.wait_for_arming_and_offboard()
            
            # Generate initial waypoint
            self.generate_next_waypoint()
            
            # Main execution loop
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                    result.exploration_completed = False
                    result.completion_reason = "Canceled by client"
                    break
                
                # Update elapsed time
                elapsed_time = time.time() - self.start_time
                
                # Check termination conditions
                if goal.max_waypoints > 0 and self.waypoints_generated >= goal.max_waypoints:
                    self.get_logger().info(f'Reached maximum waypoints: {goal.max_waypoints}')
                    result.exploration_completed = True
                    result.completion_reason = "Maximum waypoints reached"
                    break
                    
                if goal.exploration_time > 0 and elapsed_time >= goal.exploration_time:
                    self.get_logger().info(f'Reached maximum exploration time: {goal.exploration_time}s')
                    result.exploration_completed = True
                    result.completion_reason = "Maximum exploration time reached"
                    break
                
                # Check if exploration is complete (all corners visited)
                if len(self.explorer.unvisited_corners) == 0:
                    self.get_logger().info('All corners visited - exploration complete!')
                    result.exploration_completed = True
                    result.completion_reason = "All corners visited"
                    break
                
                # Publish feedback
                feedback_msg.elapsed_time = elapsed_time
                feedback_msg.waypoints_generated = self.waypoints_generated
                feedback_msg.corners_remaining = len(self.explorer.unvisited_corners)
                feedback_msg.vehicle_armed = self.vehicle_armed
                feedback_msg.offboard_mode_active = self.offboard_mode_active
                
                if self.current_waypoint is not None:
                    feedback_msg.current_waypoint.x = float(self.current_waypoint[0])
                    feedback_msg.current_waypoint.y = float(self.current_waypoint[1])
                    feedback_msg.current_waypoint.z = float(self.current_waypoint[2])
                
                if self.vehicle_position_valid:
                    feedback_msg.current_position.x = float(self.current_position[0])
                    feedback_msg.current_position.y = float(self.current_position[1])
                    feedback_msg.current_position.z = float(self.current_position[2])
                
                stats = self.explorer.get_exploration_stats()
                if stats:
                    feedback_msg.current_completion_rate = stats.get('completion_rate', 0.0)
                    feedback_msg.status_message = f"Exploring - {stats.get('corners_visited', 0)}/{self.explorer.total_corners} corners visited"
                else:
                    feedback_msg.current_completion_rate = 0.0
                    feedback_msg.status_message = "Initializing exploration"
                
                goal_handle.publish_feedback(feedback_msg)
                
                # Sleep for feedback rate
                await asyncio.sleep(0.5)  # 2 Hz feedback
        
        except Exception as e:
            self.get_logger().error(f'Error during execution: {str(e)}')
            result.exploration_completed = False
            result.completion_reason = f"Error: {str(e)}"
        
        finally:
            # Clean up timers
            trajectory_timer.destroy()
            offboard_timer.destroy()
            
            # Fill result
            stats = self.explorer.get_exploration_stats() if self.explorer else {}
            result.total_waypoints_generated = self.waypoints_generated
            result.corners_visited = stats.get('corners_visited', 0)
            result.total_corners = self.explorer.total_corners if self.explorer else 0
            result.completion_rate = stats.get('completion_rate', 0.0)
            result.total_exploration_time = time.time() - self.start_time if self.start_time else 0.0
            
            if not hasattr(result, 'exploration_completed'):
                result.exploration_completed = False
                result.completion_reason = "Unknown termination"
            
            # Clear state
            self.current_goal_handle = None
            self.current_waypoint = None
            
            self.get_logger().info('=== Path Generation Complete ===')
            self.get_logger().info(f'Waypoints generated: {result.total_waypoints_generated}')
            self.get_logger().info(f'Corners visited: {result.corners_visited}/{result.total_corners}')
            self.get_logger().info(f'Completion rate: {result.completion_rate:.1%}')
            self.get_logger().info(f'Total time: {result.total_exploration_time:.1f}s')
            self.get_logger().info(f'Completion reason: {result.completion_reason}')
            
            goal_handle.succeed()
            return result

    async def wait_for_arming_and_offboard(self, timeout=30.0):
        """Wait for vehicle to be armed and in offboard mode."""
        self.get_logger().info("Waiting for vehicle to arm and enter offboard mode...")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.vehicle_armed and self.offboard_mode_active:
                self.get_logger().info("Vehicle armed and in offboard mode")
                return True
            await asyncio.sleep(0.1)
        
        self.get_logger().warn(f"Timeout waiting for arming/offboard mode after {timeout}s")
        return False

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

        # Check if current waypoint is reached and generate next one
        if self.current_waypoint is not None and self.current_goal_handle is not None:
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

    def publish_trajectory_callback(self):
        """Publish trajectory setpoint for PX4."""
        if (
            self.current_waypoint is None
            or not self.px4_available
            or TrajectorySetpoint is None
            or self.current_goal_handle is None
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

        # Only auto arm/offboard if we have an active goal
        if self.current_goal_handle is not None:
            self.auto_arm_and_offboard()

    def generate_next_waypoint(self):
        """Generate next waypoint using butterfly explorer."""
        if self.explorer is None or self.current_goal_handle is None:
            return

        waypoint = self.explorer.generate_next_waypoint()
        self.current_waypoint = np.array(waypoint)
        self.waypoint_reached = False
        self.waypoints_generated += 1

        self.get_logger().info(
            f"New waypoint generated: [{waypoint[0]:.2f}, {waypoint[1]:.2f}, {waypoint[2]:.2f}], "
            f"Corners remaining: {len(self.explorer.unvisited_corners)}, "
            f"Total waypoints: {self.waypoints_generated}"
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


def main(args=None):
    """Main function to run the bioinspired path generator action server."""
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    node = BioinspiredPathGeneratorActionServer()
    
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
