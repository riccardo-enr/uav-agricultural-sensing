#!/usr/bin/env python3
"""Bio-inspired path generator ROS 2 action server using Levy flight patterns."""

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import asyncio

from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from uav_interfaces.action import PathGeneration
import numpy as np
import time
from .butterfly import ButterflyExplorer


class BioinspiredPathGeneratorActionServer(Node):
    """
    ROS 2 action server that generates bio-inspired flight paths using Levy flight patterns.
    Communicates with a separate UAV controller for vehicle control.
    """

    def __init__(self):
        super().__init__("bioinspired_path_generator_action_server")

        # Callback group for concurrent processing
        self.callback_group = ReentrantCallbackGroup()

        # State variables
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_waypoint = None
        self.waypoint_reached = True
        self.vehicle_position_valid = False

        # UAV state variables (from controller)
        self.vehicle_armed = False
        self.offboard_mode_active = False

        # Action execution state
        self.current_goal_handle = None
        self.explorer = None
        self.start_time = None
        self.waypoints_generated = 0
        self.visit_threshold = 1.2

        # Publishers to UAV controller
        self.waypoint_pub = self.create_publisher(
            PoseStamped,
            "/uav/waypoint",
            10
        )

        # Subscribers from UAV controller
        self.position_sub = self.create_subscription(
            PoseStamped,
            "/uav/current_pose",
            self.position_callback,
            10,
            callback_group=self.callback_group,
        )

        self.armed_sub = self.create_subscription(
            Bool,
            "/uav/armed",
            self.armed_callback,
            10,
            callback_group=self.callback_group,
        )

        # Service clients to UAV controller
        self.arm_client = self.create_client(SetBool, "/uav/arm")
        self.offboard_client = self.create_client(SetBool, "/uav/set_offboard_mode")

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

    def position_callback(self, msg):
        """Update current vehicle position from UAV controller."""
        self.current_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])
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

    def armed_callback(self, msg):
        """Update armed status from UAV controller."""
        self.vehicle_armed = msg.data

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
        
        try:
            # Automatic arming and offboard mode if enabled
            if goal.enable_auto_arm:
                await self.setup_vehicle_for_mission()
            
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

    async def setup_vehicle_for_mission(self, timeout=30.0):
        """Setup vehicle for autonomous mission (arm and set offboard mode)."""
        self.get_logger().info("Setting up vehicle for mission...")
        
        # Wait for services to be available
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Arm service not available")
            return False
            
        if not self.offboard_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Offboard service not available")
            return False
        
        # Arm the vehicle
        self.get_logger().info("Arming vehicle...")
        arm_request = SetBool.Request()
        arm_request.data = True
        
        try:
            arm_response = await self.arm_client.call_async(arm_request)
            if not arm_response.success:
                self.get_logger().warn(f"Failed to arm: {arm_response.message}")
                return False
        except Exception as e:
            self.get_logger().error(f"Arm service call failed: {e}")
            return False
        
        # Wait for arming
        start_time = time.time()
        while not self.vehicle_armed and (time.time() - start_time) < timeout:
            await asyncio.sleep(0.1)
        
        if not self.vehicle_armed:
            self.get_logger().warn("Vehicle failed to arm within timeout")
            return False
        
        # Set offboard mode
        self.get_logger().info("Setting offboard mode...")
        offboard_request = SetBool.Request()
        offboard_request.data = True
        
        try:
            offboard_response = await self.offboard_client.call_async(offboard_request)
            if not offboard_response.success:
                self.get_logger().warn(f"Failed to set offboard mode: {offboard_response.message}")
                return False
        except Exception as e:
            self.get_logger().error(f"Offboard service call failed: {e}")
            return False
        
        self.get_logger().info("Vehicle setup complete")
        return True

    def generate_next_waypoint(self):
        """Generate next waypoint using butterfly explorer and publish to UAV controller."""
        if self.explorer is None or self.current_goal_handle is None:
            return

        waypoint = self.explorer.generate_next_waypoint()
        self.current_waypoint = np.array(waypoint)
        self.waypoint_reached = False
        self.waypoints_generated += 1

        # Publish waypoint to UAV controller
        waypoint_msg = PoseStamped()
        waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_msg.header.frame_id = "map"
        waypoint_msg.pose.position.x = float(waypoint[0])
        waypoint_msg.pose.position.y = float(waypoint[1])
        waypoint_msg.pose.position.z = float(waypoint[2])
        
        self.waypoint_pub.publish(waypoint_msg)

        self.get_logger().info(
            f"New waypoint generated: [{waypoint[0]:.2f}, {waypoint[1]:.2f}, {waypoint[2]:.2f}], "
            f"Corners remaining: {len(self.explorer.unvisited_corners)}, "
            f"Total waypoints: {self.waypoints_generated}"
        )


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
