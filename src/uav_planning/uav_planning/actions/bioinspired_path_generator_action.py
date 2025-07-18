#!/usr/bin/env python3
"""Bio-inspired path generator ROS 2 action server with enhanced debugging."""

import sys
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

# Import the real ButterflyExplorer - exit if not available
try:
    from ..algorithms.butterfly import (
        TimeBasedButterflyExplorer as ButterflyExplorer,
    )

    BUTTERFLY_AVAILABLE = True
    print("✓ Real ButterflyExplorer imported successfully")
except ImportError as e:
    print(f"✗ Error: Could not import ButterflyExplorer ({e}). Exiting.")
    print("✗ Make sure the butterfly algorithm module is properly installed")
    sys.exit(1)

from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
import time

try:
    from uav_interfaces.msg import UAVState
except ImportError:
    UAVState = None
    print("Warning: UAVState not available, state machine features disabled")


class BioinspiredPathGeneratorActionServer(Node):
    """ROS 2 action server that generates bio-inspired flight paths using Levy flight patterns."""

    def __init__(self):
        super().__init__("bioinspired_path_generator_action_server")

        # Print startup info
        self.get_logger().info("=" * 50)
        self.get_logger().info(
            "STARTING BIOINSPIRED PATH GENERATOR ACTION SERVER"
        )
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"Node name: {self.get_name()}")
        self.get_logger().info(f"Real ButterflyExplorer: {BUTTERFLY_AVAILABLE}")
        self.get_logger().info(f"UAVState available: {UAVState is not None}")

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
        self.get_logger().info("Creating waypoint publisher...")
        self.waypoint_pub = self.create_publisher(
            PoseStamped, "/uav/waypoint", internal_qos
        )

        # Increase publishing rate for better responsiveness (5 Hz instead of 2 Hz)
        self.waypoint_publish_rate = 5.0
        self.waypoint_timer = self.create_timer(
            1.0 / self.waypoint_publish_rate,
            self.publish_current_waypoint,
            callback_group=self.callback_group,
        )

        # Initialize subscribers
        self.get_logger().info("Creating subscribers...")
        try:
            self.vehicle_odometry_sub = self.create_subscription(
                VehicleOdometry,
                "/fmu/out/vehicle_odometry",
                self.vehicle_odometry_callback,
                qos_profile,
                callback_group=self.callback_group,
            )

            self.vehicle_status_sub = self.create_subscription(
                VehicleStatus,
                "/fmu/out/vehicle_status_v1",
                self.vehicle_status_callback,
                qos_profile,
                callback_group=self.callback_group,
            )

            if UAVState is not None:
                self.state_sub = self.create_subscription(
                    UAVState,
                    "/uav/state",
                    self.state_callback,
                    internal_qos,
                    callback_group=self.callback_group,
                )
            else:
                self.state_sub = None

        except Exception as e:
            self.get_logger().warn(
                f"Some subscribers failed to initialize: {e}"
            )

        # Initialize state variables
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
        self.visit_threshold = (
            1.5  # Increased from 1.2 for better corner detection
        )
        self.drone_speed = (
            1.0  # CRITICAL: Increased from 0.2 to 2.0 m/s for better coverage
        )

        # Complete path planning variables
        self.planned_waypoints = []
        self.current_waypoint_index = 0
        self.path_start_time = None
        self.path_generation_active = False

        # Waypoint progression control
        self.waypoint_reached_distance = 1.0
        self.hover_duration = 0.0
        self.waypoint_reached_time = None

        self._last_arming_state = None
        self._last_nav_state = None

        # Initialize action server
        self.get_logger().info(
            "Creating action server on topic: /generate_path"
        )
        try:
            self._action_server = ActionServer(
                self,
                PathGeneration,
                "/generate_path",
                self.execute_callback,
                goal_callback=self.goal_callback,
                cancel_callback=self.cancel_callback,
                callback_group=self.callback_group,
            )
            self.get_logger().info("✓ Action server created successfully!")
        except Exception as e:
            self.get_logger().error(f"✗ Failed to create action server: {e}")
            raise

        # Initialize service clients
        self.get_logger().info("Creating service clients...")
        self.arm_client = self.create_client(SetBool, "/uav/arm")
        self.offboard_client = self.create_client(
            SetBool, "/uav/set_offboard_mode"
        )

        self.get_logger().info("✓ Action server initialization complete!")
        self.get_logger().info(f"✓ Drone speed set to: {self.drone_speed} m/s")
        self.get_logger().info(f"✓ Visit threshold: {self.visit_threshold} m")
        self.get_logger().info(
            f"✓ Waypoint publishing rate: {self.waypoint_publish_rate} Hz"
        )
        self.get_logger().info("Waiting for action clients to connect...")

        # Start a timer to periodically log server status
        self.status_timer = self.create_timer(10.0, self.log_server_status)

    def log_server_status(self):
        """Periodically log server status for debugging."""
        self.get_logger().info(f"Action server status: READY on /generate_path")
        self.get_logger().info(
            f"Current goal handle: {'Active' if self.current_goal_handle else 'None'}"
        )
        self.get_logger().info(
            f"Vehicle position valid: {self.vehicle_position_valid}"
        )
        if self.vehicle_position_valid:
            self.get_logger().info(
                f"Current position: [{self.current_position[0]:.2f}, {self.current_position[1]:.2f}, {self.current_position[2]:.2f}]"
            )

    def configure_waypoint_progression(
        self, distance_threshold=1.0, hover_duration=0.5
    ):
        """Configure waypoint progression parameters."""
        self.waypoint_reached_distance = distance_threshold
        self.hover_duration = hover_duration
        self.get_logger().info(
            f"Waypoint progression updated: distance_threshold={distance_threshold}m, "
            f"hover_duration={hover_duration}s"
        )

    def run_time_based_exploration(self, max_time, explorer):
        """Run time-based exploration for specified duration using real ButterflyExplorer."""

        self.get_logger().info("=" * 60)
        self.get_logger().info("STARTING TIME-BASED PATH PLANNING")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Duration: {max_time}s")
        self.get_logger().info(f"Drone speed: {self.drone_speed} m/s")
        self.get_logger().info(f"Visit threshold: {self.visit_threshold} m")
        self.get_logger().info(
            f"Target: Visit all {explorer.total_corners} corners of the bounding box"
        )
        self.get_logger().info(f"Algorithm: {type(explorer).__name__}")
        self.get_logger().info("-" * 60)

        waypoints = []
        current_time = 0.0

        # Add initial position as first waypoint
        waypoints.append(
            {
                "x": explorer.current_pos[0],
                "y": explorer.current_pos[1],
                "z": explorer.current_pos[2],
                "t": current_time,
                "travel_time": 0.0,
                "distance_from_prev": 0.0,
            }
        )

        waypoint_count = 0
        last_log_time = 0

        # Generate waypoints for the full duration (no artificial limits)
        while current_time < max_time:
            try:
                # Generate next waypoint using real ButterflyExplorer
                time_remaining = max_time - current_time
                next_pos = explorer.generate_next_waypoint(
                    time_remaining, max_time
                )

                # Calculate travel time to next waypoint
                distance_to_next = np.linalg.norm(
                    next_pos - explorer.current_pos
                )
                travel_time = distance_to_next / self.drone_speed

                # Check if we have enough time to reach the next waypoint
                if current_time + travel_time > max_time:
                    self.get_logger().info(
                        f"Insufficient time for next waypoint (need {travel_time:.1f}s, have {time_remaining:.1f}s)"
                    )
                    break

                # Update explorer state
                explorer.current_pos = next_pos.copy()
                current_time += travel_time
                explorer.current_time = current_time
                explorer.total_distance_traveled += distance_to_next

                # Store waypoint with timing information
                waypoints.append(
                    {
                        "x": next_pos[0],
                        "y": next_pos[1],
                        "z": next_pos[2],
                        "t": current_time,
                        "travel_time": travel_time,
                        "distance_from_prev": distance_to_next,
                    }
                )

                waypoint_count += 1

                # Log progress every 10 seconds instead of every 20 waypoints
                if current_time - last_log_time >= 10.0:
                    try:
                        unvisited = len(explorer.get_unvisited_corners())
                        stats = explorer.get_exploration_stats()
                        self.get_logger().info(
                            f"Progress: t={current_time:.1f}s/{max_time:.1f}s, "
                            f"waypoints: {waypoint_count}, "
                            f"corners: {explorer.total_corners - unvisited}/{explorer.total_corners}, "
                            f"completion: {stats.get('completion_rate', 0.0):.1%}"
                        )
                        last_log_time = current_time
                    except Exception as e:
                        self.get_logger().warn(
                            f"Error getting progress stats: {e}"
                        )

                # Safety check to prevent infinite loops (much higher limit)
                if waypoint_count > 50000:
                    self.get_logger().warn(
                        "Maximum waypoint safety limit reached (50000)"
                    )
                    break

            except Exception as e:
                self.get_logger().error(
                    f"Error generating waypoint {waypoint_count}: {e}"
                )
                break

        # Display final statistics
        try:
            stats = explorer.get_exploration_stats()
            self.get_logger().info("\n" + "=" * 60)
            self.get_logger().info("PATH PLANNING COMPLETED")
            self.get_logger().info("=" * 60)
            self.get_logger().info(
                f"Total time: {stats.get('total_time', current_time):.1f}s / {max_time}s"
            )
            self.get_logger().info(
                f"Total distance: {stats.get('total_distance', 0.0):.1f}m"
            )
            self.get_logger().info(
                f"Average speed: {stats.get('avg_speed', 0.0):.2f} m/s"
            )
            self.get_logger().info(
                f"Corners visited: {stats.get('corners_visited', 0)}/{explorer.total_corners}"
            )
            self.get_logger().info(
                f"Completion rate: {stats.get('completion_rate', 0.0):.1%}"
            )
            self.get_logger().info(f"Total waypoints: {len(waypoints)}")

            unvisited_corners = explorer.get_unvisited_corners()
            self.get_logger().info(
                f"All corners explored: {'YES' if len(unvisited_corners) == 0 else 'NO'}"
            )

            if len(unvisited_corners) > 0:
                self.get_logger().info(
                    f"Unvisited corners: {len(unvisited_corners)}"
                )

            if hasattr(stats, "get") and stats.get("corner_visit_times"):
                self.get_logger().info("\nCorner visit times:")
                for corner_idx, visit_time in sorted(
                    stats["corner_visit_times"].items()
                ):
                    self.get_logger().info(
                        f"  Corner {corner_idx}: {visit_time:.1f}s"
                    )
        except Exception as e:
            self.get_logger().warn(f"Error getting final statistics: {e}")

        return waypoints, stats

    def goal_callback(self, goal_request):
        """Accept or reject a goal."""
        self.get_logger().info("=" * 40)
        self.get_logger().info("RECEIVED GOAL REQUEST!")
        self.get_logger().info("=" * 40)
        self.get_logger().info(
            f"X bounds: [{goal_request.x_min}, {goal_request.x_max}]"
        )
        self.get_logger().info(
            f"Y bounds: [{goal_request.y_min}, {goal_request.y_max}]"
        )
        self.get_logger().info(
            f"Z bounds: [{goal_request.z_min}, {goal_request.z_max}]"
        )
        self.get_logger().info(f"Alpha: {goal_request.alpha}")
        self.get_logger().info(
            f"Visit threshold: {goal_request.visit_threshold}"
        )
        self.get_logger().info(
            f"Exploration time: {goal_request.exploration_time}"
        )

        # Basic validation
        if goal_request.x_min >= goal_request.x_max:
            self.get_logger().warn("✗ Invalid X bounds: x_min >= x_max")
            return GoalResponse.REJECT

        if goal_request.y_min >= goal_request.y_max:
            self.get_logger().warn("✗ Invalid Y bounds: y_min >= y_max")
            return GoalResponse.REJECT

        if goal_request.z_min >= goal_request.z_max:
            self.get_logger().warn("✗ Invalid Z bounds: z_min >= z_max")
            return GoalResponse.REJECT

        # Check UAV state if available
        if self.state_machine_available:
            if self.current_uav_state == 2:
                self.get_logger().warn(
                    "✗ Action already in progress, rejecting new goal"
                )
                return GoalResponse.REJECT
            elif self.current_uav_state == 4:
                self.get_logger().warn("✗ UAV is landing, rejecting goal")
                return GoalResponse.REJECT
            elif self.current_uav_state == 5:
                self.get_logger().warn(
                    "✗ UAV in emergency state, rejecting goal"
                )
                return GoalResponse.REJECT

        self.get_logger().info("✓ Goal accepted!")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle cancellation."""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the path generation action with complete path planning."""
        self.get_logger().info("=" * 50)
        self.get_logger().info("EXECUTING PATH GENERATION ACTION")
        self.get_logger().info("=" * 50)

        self.current_goal_handle = goal_handle
        goal = goal_handle.request

        # Create explorer with parameters from goal
        try:
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
            self.get_logger().info(
                "✓ Real ButterflyExplorer created successfully"
            )
        except Exception as e:
            self.get_logger().error(
                f"✗ Failed to create ButterflyExplorer: {e}"
            )
            result = PathGeneration.Result()
            result.exploration_completed = False
            result.completion_reason = f"Explorer creation failed: {str(e)}"
            goal_handle.abort()
            return result

        # Set explorer's initial position to current vehicle position if available
        if self.vehicle_position_valid:
            self.explorer.current_pos = self.current_position.copy()
            self.explorer.initial_pos = self.current_position.copy()
            self.get_logger().info(
                f"Set initial position: {self.current_position}"
            )
        else:
            self.get_logger().warn(
                "Vehicle position not available, using default [0,0,0]"
            )

        self.visit_threshold = goal.visit_threshold
        self.waypoints_generated = 0
        self.start_time = time.time()
        self.path_generation_active = True

        feedback_msg = PathGeneration.Feedback()
        result = PathGeneration.Result()

        self.get_logger().info(
            f"Starting exploration with bounds: "
            f"X[{goal.x_min}, {goal.x_max}], Y[{goal.y_min}, {goal.y_max}], Z[{goal.z_min}, {goal.z_max}]"
        )

        try:
            # Generate complete path upfront using real ButterflyExplorer
            max_time = (
                goal.exploration_time if goal.exploration_time > 0 else 600.0
            )
            self.get_logger().info(f"Generating path for {max_time}s...")

            self.planned_waypoints, path_stats = (
                self.run_time_based_exploration(max_time, self.explorer)
            )

            self.current_waypoint_index = 0
            self.path_start_time = time.time()

            self.get_logger().info(
                f"✓ Path generated with {len(self.planned_waypoints)} waypoints"
            )
            self.get_logger().info("Starting path execution...")

            # Execute the planned path
            while rclpy.ok() and self.current_waypoint_index < len(
                self.planned_waypoints
            ):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info("Goal canceled")
                    result.exploration_completed = False
                    result.completion_reason = "Canceled by client"
                    break

                elapsed_time = time.time() - self.start_time
                current_waypoint_data = self.planned_waypoints[
                    self.current_waypoint_index
                ]

                # Update current waypoint for publishing
                self.current_waypoint = np.array(
                    [
                        current_waypoint_data["x"],
                        current_waypoint_data["y"],
                        current_waypoint_data["z"],
                    ]
                )

                # Check if it's time to move to the next waypoint
                path_elapsed_time = time.time() - self.path_start_time
                if path_elapsed_time >= current_waypoint_data["t"]:
                    self.current_waypoint_index += 1
                    if self.current_waypoint_index < len(
                        self.planned_waypoints
                    ):
                        next_waypoint_data = self.planned_waypoints[
                            self.current_waypoint_index
                        ]
                        self.get_logger().info(
                            f"Moving to waypoint {self.current_waypoint_index}/{len(self.planned_waypoints)}: "
                            f"[{next_waypoint_data['x']:.2f}, {next_waypoint_data['y']:.2f}, {next_waypoint_data['z']:.2f}] "
                            f"at t={next_waypoint_data['t']:.1f}s"
                        )

                # Update feedback
                feedback_msg.elapsed_time = elapsed_time
                feedback_msg.waypoints_generated = len(self.planned_waypoints)

                # Get corners remaining
                try:
                    corners_remaining = len(
                        self.explorer.get_unvisited_corners()
                    )
                except:
                    corners_remaining = 0

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

                # Get exploration statistics
                try:
                    stats = self.explorer.get_exploration_stats()
                    if stats:
                        feedback_msg.current_completion_rate = stats.get(
                            "completion_rate", 0.0
                        )
                        feedback_msg.status_message = (
                            f"Executing planned path - Waypoint {self.current_waypoint_index}/{len(self.planned_waypoints)}, "
                            f"{stats.get('corners_visited', 0)}/{self.explorer.total_corners} corners visited "
                            f"({stats.get('completion_rate', 0.0):.1%} complete)"
                        )
                    else:
                        feedback_msg.current_completion_rate = 0.0
                        feedback_msg.status_message = "Executing planned path"
                except Exception as e:
                    self.get_logger().warn(f"Error getting explorer stats: {e}")
                    feedback_msg.current_completion_rate = 0.0
                    feedback_msg.status_message = "Executing planned path"

                goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.1)  # 10 Hz feedback instead of 2 Hz

            # Path execution completed
            if self.current_waypoint_index >= len(self.planned_waypoints):
                self.get_logger().info("✓ Planned path execution completed!")
                result.exploration_completed = True
                result.completion_reason = "Planned path completed"

        except Exception as e:
            self.get_logger().error(f"✗ Error during execution: {str(e)}")
            result.exploration_completed = False
            result.completion_reason = f"Error: {str(e)}"

        finally:
            self.path_generation_active = False
            self.planned_waypoints = []
            self.current_waypoint_index = 0
            self.waypoint_reached_time = None

            # Get final statistics
            try:
                stats = (
                    self.explorer.get_exploration_stats()
                    if self.explorer
                    else {}
                )
            except:
                stats = {}

            result.total_waypoints_generated = (
                len(self.planned_waypoints)
                if hasattr(self, "planned_waypoints")
                else 0
            )
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

            self.get_logger().info("=" * 40)
            self.get_logger().info("PATH GENERATION COMPLETE")
            self.get_logger().info("=" * 40)
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
                f"Average speed: {stats.get('avg_speed', 0.0):.2f}m/s"
            )
            self.get_logger().info(
                f"Total distance: {stats.get('total_distance', 0.0):.1f}m"
            )
            self.get_logger().info(
                f"Completion reason: {result.completion_reason}"
            )

            goal_handle.succeed()
            return result

    def vehicle_odometry_callback(self, msg):
        """Update current vehicle position from odometry."""
        self.current_position = np.array(
            [
                msg.position[1],  # Y -> X (ENU frame conversion)
                msg.position[0],  # X -> Y
                -msg.position[2],  # -Z -> Z (down -> up)
            ]
        )
        self.vehicle_position_valid = True

    def vehicle_status_callback(self, msg):
        """Update vehicle status information for arming and mode monitoring."""
        self.vehicle_armed = msg.arming_state == 2
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.offboard_mode_active = msg.nav_state == 14

        # Log state changes for debugging
        if (
            hasattr(self, "_last_arming_state")
            and self._last_arming_state != msg.arming_state
        ):
            arming_states = {1: "DISARMED", 2: "ARMED"}
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

        # Handle emergency state
        if msg.state == 5:
            if self.current_goal_handle is not None:
                self.get_logger().warn(
                    "Emergency state detected, cancelling action"
                )
                self.current_goal_handle.cancel_goal()

    def publish_current_waypoint(self):
        """Publish the current waypoint at a steady rate."""
        if self.current_waypoint is not None:
            waypoint_msg = PoseStamped()
            waypoint_msg.header.stamp = self.get_clock().now().to_msg()
            waypoint_msg.header.frame_id = "map"
            waypoint_msg.pose.position.x = float(self.current_waypoint[0])
            waypoint_msg.pose.position.y = float(self.current_waypoint[1])
            waypoint_msg.pose.position.z = float(self.current_waypoint[2])
            # Set orientation to face forward (could be improved with actual heading calculation)
            waypoint_msg.pose.orientation.w = 1.0
            self.waypoint_pub.publish(waypoint_msg)

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

    print("=" * 60)
    print("STARTING BIOINSPIRED PATH GENERATOR ACTION SERVER")
    print("=" * 60)

    try:
        executor = MultiThreadedExecutor()
        node = BioinspiredPathGeneratorActionServer()
        executor.add_node(node)

        print("Action server is ready and waiting for clients...")
        print("Use 'ros2 action list' to verify the server is running")
        print("Use 'ros2 action info /generate_path' for detailed info")
        print(
            "Use 'ros2 action send_goal /generate_path uav_interfaces/action/PathGeneration' to test"
        )

        executor.spin()
    except KeyboardInterrupt:
        print("\nShutting down action server...")
    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
