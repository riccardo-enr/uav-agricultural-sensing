#!/usr/bin/env python3
"""Simple action client for testing the bioinspired path generator action."""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from uav_interfaces.action import PathGeneration


class PathGeneratorActionClient(Node):
    """Simple action client for path generation."""

    def __init__(self):
        super().__init__('path_generator_action_client')
        self._action_client = ActionClient(self, PathGeneration, 'generate_path')

    def send_goal(self, **kwargs):
        """Send a goal to the action server."""
        goal_msg = PathGeneration.Goal()
        
        # Set default values
        goal_msg.x_min = kwargs.get('x_min', -10.0)
        goal_msg.x_max = kwargs.get('x_max', 10.0)
        goal_msg.y_min = kwargs.get('y_min', -10.0)
        goal_msg.y_max = kwargs.get('y_max', 10.0)
        goal_msg.z_min = kwargs.get('z_min', 2.0)
        goal_msg.z_max = kwargs.get('z_max', 8.0)
        goal_msg.alpha = kwargs.get('alpha', 1.5)
        goal_msg.visit_threshold = kwargs.get('visit_threshold', 1.2)
        goal_msg.velocity = kwargs.get('velocity', 5.0)
        goal_msg.enable_auto_arm = kwargs.get('enable_auto_arm', True)
        goal_msg.max_waypoints = kwargs.get('max_waypoints', 100)  # Limit for testing
        goal_msg.exploration_time = kwargs.get('exploration_time', 300.0)  # 5 minutes for testing

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info('Sending goal request...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle result."""
        result = future.result().result
        self.get_logger().info('=== Path Generation Result ===')
        self.get_logger().info(f'Total waypoints generated: {result.total_waypoints_generated}')
        self.get_logger().info(f'Corners visited: {result.corners_visited}/{result.total_corners}')
        self.get_logger().info(f'Completion rate: {result.completion_rate:.1%}')
        self.get_logger().info(f'Total exploration time: {result.total_exploration_time:.1f}s')
        self.get_logger().info(f'Exploration completed: {result.exploration_completed}')
        self.get_logger().info(f'Completion reason: {result.completion_reason}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """Handle feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Status: {feedback.status_message} | '
            f'Waypoints: {feedback.waypoints_generated} | '
            f'Remaining corners: {feedback.corners_remaining} | '
            f'Completion: {feedback.current_completion_rate:.1%} | '
            f'Time: {feedback.elapsed_time:.1f}s | '
            f'Armed: {feedback.vehicle_armed} | '
            f'Offboard: {feedback.offboard_mode_active}'
        )


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    action_client = PathGeneratorActionClient()
    
    # Send goal with custom parameters
    action_client.send_goal(
        x_min=-20.0,
        x_max=20.0,
        y_min=-20.0, 
        y_max=20.0,
        z_min=5.0,
        z_max=10.0,
        max_waypoints=50,  # Limit for demo
        exploration_time=120.0,  # 2 minutes for demo
        enable_auto_arm=False  # Disable auto-arming for safety in demo
    )

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
