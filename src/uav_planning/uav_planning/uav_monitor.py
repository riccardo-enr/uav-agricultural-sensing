#!/usr/bin/env python3
"""Simple monitoring script for the UAV system status."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool
import time
import os


class UAVMonitor(Node):
    """Monitor node for the UAV system."""

    def __init__(self):
        super().__init__("uav_monitor")
        
        # State variables
        self.current_position = None
        self.current_velocity = None
        self.current_waypoint = None
        self.armed_status = False
        
        # Timestamps for data freshness
        self.last_position_time = None
        self.last_velocity_time = None
        self.last_waypoint_time = None
        self.last_armed_time = None
        
        # Subscribers
        self.position_sub = self.create_subscription(
            PoseStamped,
            "/uav/current_pose",
            self.position_callback,
            10
        )
        
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            "/uav/current_velocity",
            self.velocity_callback,
            10
        )
        
        self.waypoint_sub = self.create_subscription(
            PoseStamped,
            "/uav/waypoint",
            self.waypoint_callback,
            10
        )
        
        self.armed_sub = self.create_subscription(
            Bool,
            "/uav/armed",
            self.armed_callback,
            10
        )
        
        # Display timer
        self.display_timer = self.create_timer(1.0, self.display_status)
        
        self.get_logger().info("UAV Monitor started - press Ctrl+C to exit")

    def position_callback(self, msg):
        """Update current position."""
        self.current_position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
        self.last_position_time = time.time()

    def velocity_callback(self, msg):
        """Update current velocity."""
        self.current_velocity = [
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z
        ]
        self.last_velocity_time = time.time()

    def waypoint_callback(self, msg):
        """Update current waypoint target."""
        self.current_waypoint = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
        self.last_waypoint_time = time.time()

    def armed_callback(self, msg):
        """Update armed status."""
        self.armed_status = msg.data
        self.last_armed_time = time.time()

    def is_data_fresh(self, timestamp, max_age=3.0):
        """Check if data is fresh (received within max_age seconds)."""
        if timestamp is None:
            return False
        return (time.time() - timestamp) < max_age

    def format_vector(self, vector, precision=2):
        """Format a 3D vector for display."""
        if vector is None:
            return "None"
        return f"[{vector[0]:.{precision}f}, {vector[1]:.{precision}f}, {vector[2]:.{precision}f}]"

    def display_status(self):
        """Display current UAV status."""
        # Clear screen (works on most terminals)
        os.system('clear' if os.name == 'posix' else 'cls')
        
        print("=" * 60)
        print("                 UAV SYSTEM MONITOR")
        print("=" * 60)
        print()
        
        # Position status
        pos_status = "✓" if self.is_data_fresh(self.last_position_time) else "✗"
        print(f"Position {pos_status}:  {self.format_vector(self.current_position)}")
        
        # Velocity status
        vel_status = "✓" if self.is_data_fresh(self.last_velocity_time) else "✗"
        vel_magnitude = 0.0
        if self.current_velocity:
            vel_magnitude = (sum(v**2 for v in self.current_velocity)) ** 0.5
        print(f"Velocity {vel_status}:  {self.format_vector(self.current_velocity)} (|v|={vel_magnitude:.2f})")
        
        # Waypoint status
        wp_status = "✓" if self.is_data_fresh(self.last_waypoint_time) else "✗"
        print(f"Waypoint {wp_status}:  {self.format_vector(self.current_waypoint)}")
        
        # Armed status
        armed_status = "✓" if self.is_data_fresh(self.last_armed_time) else "✗"
        armed_text = "ARMED" if self.armed_status else "DISARMED"
        print(f"Armed {armed_status}:     {armed_text}")
        
        print()
        
        # Distance to waypoint
        if self.current_position and self.current_waypoint:
            distance = sum((p - w)**2 for p, w in zip(self.current_position, self.current_waypoint)) ** 0.5
            print(f"Distance to waypoint: {distance:.2f} m")
        else:
            print("Distance to waypoint: N/A")
        
        print()
        print("-" * 60)
        print("Data Freshness Legend:")
        print("  ✓ = Fresh (< 3 seconds old)")
        print("  ✗ = Stale or missing")
        print()
        print("Press Ctrl+C to exit")
        print("-" * 60)


def main(args=None):
    """Main function to run the monitor."""
    rclpy.init(args=args)
    
    node = UAVMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down UAV Monitor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
