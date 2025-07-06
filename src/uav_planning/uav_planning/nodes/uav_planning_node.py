#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# Import the core logic from the algorithms module
# Import the ButterflyGenerator from the reorganized module structure
from ..algorithms.butterfly_generator import ButterflyGenerator


class ButterflyPathNode(Node):
    """
    A ROS 2 node that wraps the ButterflyGenerator. It handles parameter
    loading, message publishing, and other ROS-specific tasks.
    """

    def __init__(self):
        super().__init__("butterfly_path_node")

        # --- Declare and Get ROS 2 Parameters ---
        self.declare_parameter("x_min", 0.0)
        self.declare_parameter("x_max", 10.0)
        self.declare_parameter("y_min", 0.0)
        self.declare_parameter("y_max", 5.0)
        self.declare_parameter("z_min", 1.0)
        self.declare_parameter("z_max", 2.0)
        self.declare_parameter("alpha", 1.5)
        self.declare_parameter("step_scale", 1.0)

        # --- Instantiate the Core Logic from the imported class ---
        self.generator = ButterflyGenerator(
            x_min=self.get_parameter("x_min")
            .get_parameter_value()
            .double_value,
            x_max=self.get_parameter("x_max")
            .get_parameter_value()
            .double_value,
            y_min=self.get_parameter("y_min")
            .get_parameter_value()
            .double_value,
            y_max=self.get_parameter("y_max")
            .get_parameter_value()
            .double_value,
            z_min=self.get_parameter("z_min")
            .get_parameter_value()
            .double_value,
            z_max=self.get_parameter("z_max")
            .get_parameter_value()
            .double_value,
            alpha=self.get_parameter("alpha")
            .get_parameter_value()
            .double_value,
            step_scale=self.get_parameter("step_scale")
            .get_parameter_value()
            .double_value,
        )

        # --- ROS 2 Comms ---
        self.publisher_ = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10
        )
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz timer

        self.get_logger().info("ROS 2 Butterfly Path Planner node started.")

    def timer_callback(self):
        """Called by the timer to generate and publish a new waypoint."""
        # Get the next waypoint from our generator instance
        x, y, z = self.generator.generate_next_waypoint()

        # Create and populate the ROS 2 message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.w = 1.0  # Neutral orientation

        # Publish the message
        self.publisher_.publish(pose_msg)
        self.get_logger().info(f"Sending waypoint: ({x:.2f}, {y:.2f}, {z:.2f})")


def main(args=None):
    """
    Main function to initialize and run the ROS 2 node.
    """
    rclpy.init(args=args)
    node = ButterflyPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Proper cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
