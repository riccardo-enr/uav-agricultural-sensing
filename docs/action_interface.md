# Bioinspired Path Generator Action

This document describes the ROS 2 action interface for the bioinspired path generator, which converts the original node-based implementation into an action server/client architecture.

## Action Definition

The action is defined in `uav_interfaces/action/PathGeneration.action` and provides the following interface:

### Goal

The goal message contains parameters for configuring the path generation:

```
float64 x_min -10.0          # Minimum X boundary (meters)
float64 x_max 10.0           # Maximum X boundary (meters)  
float64 y_min -10.0          # Minimum Y boundary (meters)
float64 y_max 10.0           # Maximum Y boundary (meters)
float64 z_min 2.0            # Minimum Z boundary (meters)
float64 z_max 8.0            # Maximum Z boundary (meters)
float64 alpha 1.5            # Levy flight alpha parameter
float64 visit_threshold 1.2  # Distance threshold for waypoint completion (meters)
float64 velocity 5.0         # Desired UAV velocity (m/s)
bool enable_auto_arm true    # Enable automatic arming and offboard mode
uint32 max_waypoints 1000    # Maximum waypoints to generate (0 = unlimited)
float64 exploration_time 0.0 # Maximum exploration time (seconds, 0 = unlimited)
```

### Result

The result message provides final exploration statistics:

```
uint32 total_waypoints_generated  # Total waypoints generated during exploration
uint32 corners_visited           # Number of corners that were visited
uint32 total_corners            # Total number of corners in the exploration area
float64 completion_rate         # Percentage of area explored (0.0 to 1.0)
float64 total_exploration_time  # Total time spent exploring (seconds)
bool exploration_completed      # Whether exploration completed successfully
string completion_reason        # Reason for exploration termination
```

### Feedback

The feedback message provides real-time exploration status:

```
geometry_msgs/Point current_waypoint    # Current target waypoint
geometry_msgs/Point current_position    # Current UAV position
uint32 waypoints_generated             # Waypoints generated so far
uint32 corners_remaining               # Unexplored corners remaining
float64 current_completion_rate        # Current exploration completion rate
float64 elapsed_time                   # Time elapsed since start (seconds)
bool vehicle_armed                     # Whether UAV is armed
bool offboard_mode_active             # Whether UAV is in offboard mode
string status_message                  # Human-readable status description
```

## Usage

### Running the Action Server

Start the action server:

```bash
ros2 run uav_planning bioinspired_planner_action
```

Or using the launch file:

```bash
ros2 launch uav_planning path_generator_action.launch.py
```

### Running the Action Client

Start the test client:

```bash
ros2 run uav_planning path_generator_client
```

Or run the complete demo with both server and client:

```bash
ros2 launch uav_planning path_generator_demo.launch.py
```

### Custom Action Client

You can create custom action clients in Python:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from uav_interfaces.action import PathGeneration

class MyPathClient(Node):
    def __init__(self):
        super().__init__('my_path_client')
        self._action_client = ActionClient(self, PathGeneration, 'generate_path')

    def send_goal(self):
        goal_msg = PathGeneration.Goal()
        goal_msg.x_min = -50.0
        goal_msg.x_max = 50.0
        goal_msg.y_min = -50.0
        goal_msg.y_max = 50.0
        goal_msg.z_min = 5.0
        goal_msg.z_max = 15.0
        goal_msg.max_waypoints = 200
        goal_msg.exploration_time = 600.0  # 10 minutes
        
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print(f"Progress: {feedback.current_completion_rate:.1%}")
```

### Command Line Interface

You can also use the command line to interact with the action:

```bash
# Send a goal
ros2 action send_goal /generate_path uav_interfaces/action/PathGeneration "
{
  x_min: -20.0,
  x_max: 20.0,
  y_min: -20.0,
  y_max: 20.0,
  z_min: 3.0,
  z_max: 10.0,
  max_waypoints: 100,
  exploration_time: 300.0,
  enable_auto_arm: false
}"

# List active actions
ros2 action list

# Get action info
ros2 action info /generate_path
```

## Advantages of Action Interface

The action interface provides several advantages over the original node-based approach:

1. **Goal-oriented**: Clients can specify exploration parameters and receive completion notifications
2. **Feedback**: Real-time progress updates during exploration
3. **Cancellation**: Clients can cancel ongoing explorations
4. **Result reporting**: Detailed statistics upon completion
5. **Multiple clients**: Multiple clients can queue exploration requests
6. **Standard interface**: Uses ROS 2 action conventions for better integration

## Integration with PX4

The action server maintains the same PX4 integration as the original node:

- Publishes trajectory setpoints to `/fmu/in/trajectory_setpoint`
- Publishes offboard control mode to `/fmu/in/offboard_control_mode`
- Subscribes to vehicle odometry from `/fmu/out/vehicle_odometry`
- Subscribes to vehicle status from `/fmu/out/vehicle_status_v1`
- Supports automatic arming and offboard mode switching

## Configuration

The action server uses the same bio-inspired exploration algorithm but now allows per-goal configuration through the action interface rather than ROS parameters. This provides more flexibility for different exploration scenarios.

## Safety Considerations

- Set `enable_auto_arm: false` for testing without actual UAV hardware
- Use reasonable bounds for exploration area
- Set appropriate `max_waypoints` and `exploration_time` limits
- Monitor feedback for vehicle status before enabling auto-arming
- Always have manual override capabilities when working with real UAVs
