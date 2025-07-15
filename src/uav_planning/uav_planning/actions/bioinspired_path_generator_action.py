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

from px4_msgs.msg import (
    VehicleOdometry,
    VehicleStatus,
)

from uav_interfaces.action import PathGeneration
import numpy as np
import time
from ..algorithms.butterfly import (
    TimeBasedButterflyExplorer as ButterflyExplorer,
)
from std_srvs.srv import SetBool
from geometry_msgs.msg import Point

try:
    from uav_interfaces.msg import UAVState
except ImportError:
    UAVState = None


class BioinspiredPathGeneratorActionServer(Node):
    """ROS 2 action server that generates bio-inspired flight paths using Levy flight patterns."""

    def __init__(self):
        super().__init__("bioinspired_path_generator_action_server")
        self.callback_group = ReentrantCallbackGroup()

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

        # Add waypoint publisher
        self.waypoint_pub = self.create_publisher(
            Point, "/uav/waypoint", internal_qos
        )

        # Add timer for steady waypoint publishing (default 2 Hz)
        self.waypoint_publish_rate = 2.0
        self.waypoint_timer = self.create_timer(
            1.0 / self.waypoint_publish_rate,
            self.publish_current_waypoint,
            callback_group=self.callback_group,
        )

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

        self.state_sub = self.create_subscription(
            UAVState,
            "/uav/state",
            self.state_callback,
            internal_qos,
            callback_group=self.callback_group,
        )

        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_waypoint = None
        self.waypoint_reached = True
        self.vehicle_position_valid = False

        self.vehicle_armed = False
        self.offboard_mode_active = False
        self.nav_state = 0
        self.arming_state = 0
        self.offboard_setpoint_counter = 0

        self.current_uav_state = 0
        self.state_machine_available = UAVState is not None

        self.current_goal_handle = None
        self.explorer = None
        self.start_time = None
        self.waypoints_generated = 0
        self.trajectory_publish_rate = 10.0
        self.visit_threshold = 1.2
        self.drone_speed = 0.5

        self._last_arming_state = None
        self._last_nav_state = None

        self._action_server = ActionServer(
            self,
            PathGeneration,
            "/generate_path",
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        self.arm_client = self.create_client(SetBool, "/uav/arm")
        self.offboard_client = self.create_client(
            SetBool, "/uav/set_offboard_mode"
        )

        self.get_logger().info(
            "Bioinspired path generator action server initialized"
        )

    def goal_callback(self, goal_request):
        """Accept or reject a goal."""
        self.get_logger().info("Received goal request")

        if goal_request.x_min >= goal_request.x_max:
            self.get_logger().warn("Invalid X bounds: x_min >= x_max")
            return GoalResponse.REJECT

        if goal_request.y_min >= goal_request.y_max:
            self.get_logger().warn("Invalid Y bounds: y_min >= y_max")
            return GoalResponse.REJECT

        if goal_request.z_min >= goal_request.z_max:
            self.get_logger().warn("Invalid Z bounds: z_min >= z_max")
            return GoalResponse.REJECT

        if self.state_machine_available:
            if self.current_uav_state == 2:
                self.get_logger().warn(
                    "Action already in progress, rejecting new goal"
                )
                return GoalResponse.REJECT
            elif self.current_uav_state == 4:
                self.get_logger().warn("UAV is landing, rejecting goal")
                return GoalResponse.REJECT
            elif self.current_uav_state == 5:
                self.get_logger().warn("UAV in emergency state, rejecting goal")
                return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle cancellation."""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the path generation action."""
        self.get_logger().info("Executing path generation...")

        self.current_goal_handle = goal_handle
        goal = goal_handle.request

        self.explorer = ButterflyExplorer(
            x_min=goal.x_min,
            x_max=goal.x_max,
            y_min=goal.y_min,
            y_max=goal.y_max,
            z_min=goal.z_min,
            z_max=goal.z_max,
            alpha=goal.alpha,
            visit_threshold=goal.visit_threshold,
            drone_speed=self.drone_speed,
        )

        self.visit_threshold = goal.visit_threshold
        self.waypoints_generated = 0
        self.start_time = time.time()

        feedback_msg = PathGeneration.Feedback()
        result = PathGeneration.Result()

        self.get_logger().info(
            f"Starting exploration with bounds: "
            f"X[{goal.x_min}, {goal.x_max}], Y[{goal.y_min}, {goal.y_max}], Z[{goal.z_min}, {goal.z_max}]"
        )

        try:
            self.generate_next_waypoint()

            self.get_logger().info("Sending initial trajectory setpoints...")
            for i in range(20):
                time.sleep(0.1)

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info("Goal canceled")
                    result.exploration_completed = False
                    result.completion_reason = "Canceled by client"
                    break

                elapsed_time = time.time() - self.start_time

                if (
                    goal.max_waypoints > 0
                    and self.waypoints_generated >= goal.max_waypoints
                ):
                    self.get_logger().info(
                        f"Reached maximum waypoints: {goal.max_waypoints}"
                    )
                    result.exploration_completed = True
                    result.completion_reason = "Maximum waypoints reached"
                    break

                if (
                    goal.exploration_time > 0
                    and elapsed_time >= goal.exploration_time
                ):
                    self.get_logger().info(
                        f"Reached maximum exploration time: {goal.exploration_time}s"
                    )
                    result.exploration_completed = True
                    result.completion_reason = (
                        "Maximum exploration time reached"
                    )
                    break

                if hasattr(self.explorer, "get_unvisited_corners"):
                    corners_remaining = len(
                        self.explorer.get_unvisited_corners()
                    )
                else:
                    corners_remaining = 0

                if corners_remaining == 0:
                    self.get_logger().info(
                        "All corners visited - exploration complete!"
                    )
                    result.exploration_completed = True
                    result.completion_reason = "All corners visited"
                    break

                feedback_msg.elapsed_time = elapsed_time
                feedback_msg.waypoints_generated = self.waypoints_generated
                feedback_msg.corners_remaining = corners_remaining
                feedback_msg.vehicle_armed = self.vehicle_armed
                feedback_msg.offboard_mode_active = self.offboard_mode_active

                if self.current_waypoint is not None:
                    feedback_msg.current_waypoint.x = float(
                        self.current_waypoint[0]
                    )
                    feedback_msg.current_waypoint.y = float(
                        self.current_waypoint[1]
                    )
                    feedback_msg.current_waypoint.z = float(
                        self.current_waypoint[2]
                    )

                if self.vehicle_position_valid:
                    feedback_msg.current_position.x = float(
                        self.current_position[0]
                    )
                    feedback_msg.current_position.y = float(
                        self.current_position[1]
                    )
                    feedback_msg.current_position.z = float(
                        self.current_position[2]
                    )

                stats = self.explorer.get_exploration_stats()
                if stats:
                    feedback_msg.current_completion_rate = stats.get(
                        "completion_rate", 0.0
                    )
                    feedback_msg.status_message = f"Exploring - {stats.get('corners_visited', 0)}/{self.explorer.total_corners} corners visited"
                else:
                    feedback_msg.current_completion_rate = 0.0
                    feedback_msg.status_message = "Initializing exploration"

                goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.5)

        except Exception as e:
            self.get_logger().error(f"Error during execution: {str(e)}")
            result.exploration_completed = False
            result.completion_reason = f"Error: {str(e)}"

        finally:
            stats = (
                self.explorer.get_exploration_stats() if self.explorer else {}
            )
            result.total_waypoints_generated = self.waypoints_generated
            result.corners_visited = stats.get("corners_visited", 0)
            result.total_corners = (
                self.explorer.total_corners if self.explorer else 0
            )
            result.completion_rate = stats.get("completion_rate", 0.0)
            result.total_exploration_time = (
                time.time() - self.start_time if self.start_time else 0.0
            )

            if not hasattr(result, "exploration_completed"):
                result.exploration_completed = False
                result.completion_reason = "Unknown termination"

            self.current_goal_handle = None
            self.current_waypoint = None

            self.get_logger().info("=== Path Generation Complete ===")
            self.get_logger().info(
                f"Waypoints generated: {result.total_waypoints_generated}"
            )
            self.get_logger().info(
                f"Corners visited: {result.corners_visited}/{result.total_corners}"
            )
            self.get_logger().info(
                f"Completion rate: {result.completion_rate:.1%}"
            )
            self.get_logger().info(
                f"Total time: {result.total_exploration_time:.1f}s"
            )
            self.get_logger().info(
                f"Completion reason: {result.completion_reason}"
            )

            goal_handle.succeed()
            return result

    def vehicle_odometry_callback(self, msg):
        """Update current vehicle position from odometry and generate next waypoint if needed."""
        self.current_position = np.array(
            [
                msg.position[1],
                msg.position[0],
                -msg.position[2],
            ]
        )
        self.vehicle_position_valid = True

        if hasattr(self.explorer, "current_pos") and self.explorer is not None:
            distance_traveled = np.linalg.norm(
                self.current_position - self.explorer.current_pos
            )
            self.explorer.total_distance_traveled += float(distance_traveled)
            self.explorer.current_pos = self.current_position

            if self.start_time is not None:
                elapsed_time = time.time() - self.start_time
                self.explorer.current_time = elapsed_time

        if (
            self.current_waypoint is not None
            and self.current_goal_handle is not None
        ):
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
        self.vehicle_armed = msg.arming_state == 2
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.offboard_mode_active = msg.nav_state == 14

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

        if msg.state == 5:
            if self.current_goal_handle is not None:
                self.get_logger().warn(
                    "Emergency state detected, cancelling action"
                )
                self.current_goal_handle.cancel_goal()

    def publish_current_waypoint(self):
        """Publish the current waypoint at a steady rate."""
        if self.current_waypoint is not None:
            waypoint_msg = Point()
            waypoint_msg.x = float(self.current_waypoint[0])
            waypoint_msg.y = float(self.current_waypoint[1])
            waypoint_msg.z = float(self.current_waypoint[2])
            self.waypoint_pub.publish(waypoint_msg)

    def generate_next_waypoint(self):
        """Generate next waypoint using butterfly explorer with time constraints."""
        if self.explorer is None or self.current_goal_handle is None:
            return

        goal = self.current_goal_handle.request
        elapsed_time = time.time() - self.start_time if self.start_time else 0.0
        time_remaining = (
            max(0, goal.exploration_time - elapsed_time)
            if goal.exploration_time > 0
            else goal.exploration_time
        )

        waypoint = self.explorer.generate_next_waypoint(
            time_remaining, goal.exploration_time
        )
        self.current_waypoint = np.array(waypoint)
        self.waypoint_reached = False
        self.waypoints_generated += 1

        # Publish waypoint reference
        waypoint_msg = Point()
        waypoint_msg.x = float(waypoint[0])
        waypoint_msg.y = float(waypoint[1])
        waypoint_msg.z = float(waypoint[2])
        self.waypoint_pub.publish(waypoint_msg)

        if hasattr(self.explorer, "get_unvisited_corners"):
            corners_remaining = len(self.explorer.get_unvisited_corners())
        else:
            corners_remaining = 0

        stats = self.explorer.get_exploration_stats()

        self.get_logger().info(
            f"New waypoint generated: [{waypoint[0]:.2f}, {waypoint[1]:.2f}, {waypoint[2]:.2f}], "
            f"Corners remaining: {corners_remaining}, "
            f"Time remaining: {time_remaining:.1f}s, "
            f"Completion: {stats.get('completion_rate', 0):.1%}"
        )

    def call_arm_service(self, arm: bool):
        """Call the UAV controller arm service."""
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Arm service not available")
            return False

        request = SetBool.Request()
        request.data = arm

        future = self.arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result is not None:
            return result.success
        return False

    def call_offboard_service(self, offboard: bool):
        """Call the UAV controller offboard service."""
        if not self.offboard_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Offboard service not available")
            return False

        request = SetBool.Request()
        request.data = offboard

        future = self.offboard_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result is not None:
            return result.success
        return False


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
