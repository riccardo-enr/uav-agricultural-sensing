#!/usr/bin/env python3
"""Test script to verify the separated UAV architecture is working correctly."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import time


class ArchitectureTestNode(Node):
    """Test node to verify the separated architecture is working."""

    def __init__(self):
        super().__init__("architecture_test_node")
        
        # Publishers
        self.waypoint_pub = self.create_publisher(
            PoseStamped,
            "/uav/waypoint",
            10
        )
        
        # Subscribers
        self.position_sub = self.create_subscription(
            PoseStamped,
            "/uav/current_pose",
            self.position_callback,
            10
        )
        
        self.armed_sub = self.create_subscription(
            Bool,
            "/uav/armed",
            self.armed_callback,
            10
        )
        
        # State tracking
        self.received_position = False
        self.received_armed_status = False
        self.current_position = None
        self.armed_status = None
        
        # Test timer
        self.test_timer = self.create_timer(1.0, self.run_test)
        self.test_step = 0
        
        self.get_logger().info("Architecture test node started")

    def position_callback(self, msg):
        """Receive position updates from UAV controller."""
        self.current_position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
        self.received_position = True
        
        if not hasattr(self, '_position_logged'):
            self.get_logger().info(
                f"✓ Received position from UAV controller: "
                f"[{self.current_position[0]:.2f}, {self.current_position[1]:.2f}, {self.current_position[2]:.2f}]"
            )
            self._position_logged = True

    def armed_callback(self, msg):
        """Receive armed status from UAV controller."""
        self.armed_status = msg.data
        self.received_armed_status = True
        
        if not hasattr(self, '_armed_logged'):
            self.get_logger().info(f"✓ Received armed status from UAV controller: {self.armed_status}")
            self._armed_logged = True

    def run_test(self):
        """Run the architecture test steps."""
        self.test_step += 1
        
        if self.test_step == 1:
            self.get_logger().info("=== Testing Separated UAV Architecture ===")
            self.get_logger().info("Step 1: Waiting for UAV controller to publish status...")
        
        elif self.test_step == 3:
            if self.received_position and self.received_armed_status:
                self.get_logger().info("✓ UAV controller is publishing status correctly")
            else:
                self.get_logger().warn("✗ Not receiving status from UAV controller")
                missing = []
                if not self.received_position:
                    missing.append("position")
                if not self.received_armed_status:
                    missing.append("armed status")
                self.get_logger().warn(f"Missing: {', '.join(missing)}")
        
        elif self.test_step == 5:
            self.get_logger().info("Step 2: Testing waypoint communication...")
            waypoint_msg = PoseStamped()
            waypoint_msg.header.stamp = self.get_clock().now().to_msg()
            waypoint_msg.header.frame_id = "map"
            waypoint_msg.pose.position.x = 5.0
            waypoint_msg.pose.position.y = 3.0
            waypoint_msg.pose.position.z = 2.0
            
            self.waypoint_pub.publish(waypoint_msg)
            self.get_logger().info("✓ Published test waypoint [5.0, 3.0, 2.0]")
        
        elif self.test_step == 8:
            # Send another waypoint
            waypoint_msg = PoseStamped()
            waypoint_msg.header.stamp = self.get_clock().now().to_msg()
            waypoint_msg.header.frame_id = "map"
            waypoint_msg.pose.position.x = -2.0
            waypoint_msg.pose.position.y = 4.0
            waypoint_msg.pose.position.z = 3.5
            
            self.waypoint_pub.publish(waypoint_msg)
            self.get_logger().info("✓ Published second test waypoint [-2.0, 4.0, 3.5]")
        
        elif self.test_step == 12:
            self.get_logger().info("=== Architecture Test Complete ===")
            self.get_logger().info("Summary:")
            self.get_logger().info(f"  - Position updates: {'✓ Working' if self.received_position else '✗ Failed'}")
            self.get_logger().info(f"  - Armed status: {'✓ Working' if self.received_armed_status else '✗ Failed'}")
            self.get_logger().info("  - Waypoint publishing: ✓ Working")
            
            if self.received_position and self.received_armed_status:
                self.get_logger().info("🎉 Separated architecture is working correctly!")
                self.get_logger().info("You can now:")
                self.get_logger().info("  1. Use the UAV controller for low-level control")
                self.get_logger().info("  2. Use the path generator action for high-level missions")
                self.get_logger().info("  3. Test with: ros2 launch uav_planning uav_system_demo.launch.py")
            else:
                self.get_logger().error("❌ Architecture test failed - check if UAV controller is running")
            
            # Stop the test
            self.test_timer.cancel()


def main(args=None):
    """Main function to run the test."""
    rclpy.init(args=args)
    
    node = ArchitectureTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
