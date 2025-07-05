# Code Reference

This document provides a comprehensive reference for the UAV Agricultural Sensing codebase, including all classes, methods, and ROS 2 interfaces.

## Core Classes

### ButterflyExplorer

The main algorithm implementation for bio-inspired path planning.

```python
class ButterflyExplorer:
    """
    Bio-inspired path planning using Lévy flight patterns.
    
    This class implements a butterfly-inspired exploration algorithm that generates
    waypoints following Lévy flight distributions for efficient area coverage.
    """
```

#### Constructor

```python
def __init__(self, x_min: float, x_max: float, y_min: float, y_max: float,
             z_min: float, z_max: float, alpha: float = 1.5,
             visit_threshold: float = 1.2) -> None:
    """
    Initialize the butterfly explorer.
    
    Args:
        x_min: Minimum X boundary (meters)
        x_max: Maximum X boundary (meters)
        y_min: Minimum Y boundary (meters)
        y_max: Maximum Y boundary (meters)
        z_min: Minimum altitude (meters)
        z_max: Maximum altitude (meters)
        alpha: Lévy flight shape parameter (1.0 < α ≤ 3.0)
        visit_threshold: Distance threshold for waypoint completion (meters)
    
    Raises:
        ValueError: If parameters are invalid (e.g., min >= max)
    """
```

#### Core Methods

##### generate_next_waypoint()

```python
def generate_next_waypoint(self) -> Tuple[float, float, float]:
    """
    Generate the next waypoint using Lévy flight pattern.
    
    Returns:
        Tuple[float, float, float]: Next waypoint as (x, y, z) coordinates
    
    Raises:
        RuntimeError: If boundary constraints cannot be satisfied
    """
```

##### update_position()

```python
def update_position(self, x: float, y: float, z: float) -> None:
    """
    Update the current position of the explorer.
    
    Args:
        x: Current X position (meters)
        y: Current Y position (meters)
        z: Current Z position (meters)
    """
```

##### get_exploration_stats()

```python
def get_exploration_stats(self) -> Dict[str, Any]:
    """
    Get current exploration statistics.
    
    Returns:
        Dict containing:
        - corners_visited: Number of boundary corners visited
        - total_corners: Total number of boundary corners
        - completion_rate: Exploration completion percentage (0-1)
        - total_steps: Total number of waypoints generated
        - average_step_length: Average distance between waypoints
    """
```

##### reset_exploration()

```python
def reset_exploration(self) -> None:
    """
    Reset exploration state to start over.
    
    Clears all exploration history and reinitializes corner tracking.
    """
```

#### Properties

```python
@property
def current_position(self) -> Optional[Tuple[float, float, float]]:
    """Current position as (x, y, z) tuple or None if not set."""

@property
def boundaries(self) -> Dict[str, float]:
    """Boundary constraints as dictionary with keys: x_min, x_max, y_min, y_max, z_min, z_max."""

@property
def unvisited_corners(self) -> List[Tuple[float, float, float]]:
    """List of boundary corners not yet visited."""

@property
def total_corners(self) -> int:
    """Total number of boundary corners (always 8 for rectangular boundary)."""
```

### BioinspiredPathGenerator

The main ROS 2 node that integrates the algorithm with the ROS ecosystem.

```python
class BioinspiredPathGenerator(Node):
    """
    ROS 2 node for bio-inspired UAV path planning.
    
    Integrates ButterflyExplorer algorithm with ROS 2 and PX4 autopilot systems.
    Publishes trajectory setpoints and manages UAV state.
    """
```

#### Constructor

```python
def __init__(self) -> None:
    """
    Initialize the bioinspired path generator node.
    
    Sets up ROS 2 publishers, subscribers, parameters, and timers.
    Initializes the ButterflyExplorer algorithm with configured parameters.
    """
```

#### Callback Methods

##### vehicle_odometry_callback()

```python
def vehicle_odometry_callback(self, msg: VehicleOdometry) -> None:
    """
    Process vehicle odometry messages.
    
    Args:
        msg: VehicleOdometry message from PX4
        
    Updates current position and triggers waypoint generation if needed.
    """
```

##### vehicle_status_callback()

```python
def vehicle_status_callback(self, msg: VehicleStatus) -> None:
    """
    Process vehicle status messages.
    
    Args:
        msg: VehicleStatus message from PX4
        
    Updates arming state and navigation mode information.
    """
```

##### publish_trajectory_callback()

```python
def publish_trajectory_callback(self) -> None:
    """
    Timer callback to publish trajectory setpoints.
    
    Publishes current waypoint as TrajectorySetpoint message to PX4.
    Called at trajectory_publish_rate frequency.
    """
```

#### Control Methods

##### generate_next_waypoint()

```python
def generate_next_waypoint(self) -> None:
    """
    Generate and set next waypoint using the explorer algorithm.
    
    Updates internal state and publishes debug information.
    """
```

##### arm_vehicle()

```python
def arm_vehicle(self) -> None:
    """
    Send arm command to the vehicle.
    
    Publishes VehicleCommand message to arm the UAV.
    """
```

##### set_offboard_mode()

```python
def set_offboard_mode(self) -> None:
    """
    Switch vehicle to offboard control mode.
    
    Publishes VehicleCommand message to enable offboard control.
    """
```

##### reset_exploration()

```python
def reset_exploration(self) -> None:
    """
    Reset the exploration algorithm.
    
    Reinitializes the ButterflyExplorer and clears current waypoint.
    """
```

#### Utility Methods

##### get_exploration_stats()

```python
def get_exploration_stats(self) -> Dict[str, Any]:
    """
    Get current exploration statistics from the algorithm.
    
    Returns:
        Dictionary with exploration metrics
    """
```

##### get_vehicle_status()

```python
def get_vehicle_status(self) -> Dict[str, Any]:
    """
    Get current vehicle status information.
    
    Returns:
        Dictionary containing:
        - armed: Boolean arming state
        - offboard_active: Boolean offboard mode state
        - nav_state: Navigation state code
        - arming_state: Arming state code
        - setpoint_counter: Offboard setpoint counter
        - px4_available: Boolean PX4 message availability
    """
```

## ROS 2 Interface

### Topics

#### Published Topics

| Topic                           | Message Type                   | Description                   |
| ------------------------------- | ------------------------------ | ----------------------------- |
| `/fmu/in/trajectory_setpoint`   | `px4_msgs/TrajectorySetpoint`  | Waypoint commands to PX4      |
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | Control mode settings         |
| `/fmu/in/vehicle_command`       | `px4_msgs/VehicleCommand`      | Vehicle commands (arm/disarm) |
| `~/current_waypoint`            | `geometry_msgs/Point`          | Current target waypoint       |
| `~/computation_time`            | `std_msgs/Float64`             | Algorithm computation time    |
| `/debug/corners_remaining`      | `std_msgs/Int32`               | Unvisited corners count       |

#### Subscribed Topics

| Topic                       | Message Type               | Description               |
| --------------------------- | -------------------------- | ------------------------- |
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | Vehicle position feedback |
| `/fmu/out/vehicle_status`   | `px4_msgs/VehicleStatus`   | Vehicle state information |

### Parameters

#### Boundary Parameters

| Parameter | Type   | Default | Description                 |
| --------- | ------ | ------- | --------------------------- |
| `x_min`   | double | -10.0   | Minimum X boundary (meters) |
| `x_max`   | double | 10.0    | Maximum X boundary (meters) |
| `y_min`   | double | -10.0   | Minimum Y boundary (meters) |
| `y_max`   | double | 10.0    | Maximum Y boundary (meters) |
| `z_min`   | double | 2.0     | Minimum altitude (meters)   |
| `z_max`   | double | 8.0     | Maximum altitude (meters)   |

#### Algorithm Parameters

| Parameter         | Type   | Default | Description                      |
| ----------------- | ------ | ------- | -------------------------------- |
| `alpha`           | double | 1.5     | Lévy flight shape parameter      |
| `visit_threshold` | double | 1.2     | Waypoint reach distance (meters) |
| `velocity`        | double | 5.0     | UAV cruise velocity (m/s)        |

#### Timing Parameters

| Parameter                  | Type   | Default | Description                          |
| -------------------------- | ------ | ------- | ------------------------------------ |
| `waypoint_generation_rate` | double | 1.0     | Waypoint generation frequency (Hz)   |
| `trajectory_publish_rate`  | double | 10.0    | Trajectory publishing frequency (Hz) |

#### Control Parameters

| Parameter                | Type | Default | Description                    |
| ------------------------ | ---- | ------- | ------------------------------ |
| `enable_path_generation` | bool | true    | Enable/disable path generation |

### Services

Currently no custom services are implemented. The node uses ROS 2 parameter services for runtime configuration.

### Actions

Currently no actions are implemented. Future versions may include mission-level actions.

## Message Definitions

### Custom Messages

The package uses standard ROS 2 and px4_msgs message types. No custom messages are defined.

### Key Message Types Used

#### px4_msgs/TrajectorySetpoint

```
# Trajectory setpoint for PX4 autopilot
uint64 timestamp
float32[3] position    # Position setpoint in NED frame [m]
float32[3] velocity    # Velocity setpoint in NED frame [m/s]
float32[3] acceleration # Acceleration setpoint in NED frame [m/s^2]
float32 yaw            # Yaw setpoint [rad]
float32 yawspeed       # Yaw rate setpoint [rad/s]
```

#### px4_msgs/VehicleOdometry

```
# Vehicle odometry data
uint64 timestamp
uint64 timestamp_sample
uint8 pose_frame
uint8 velocity_frame
float32[3] position    # Position in NED frame [m]
float32[4] q           # Quaternion orientation
float32[3] velocity    # Velocity in NED frame [m/s]
float32[3] angular_velocity # Angular velocity [rad/s]
float32[21] pose_covariance
float32[21] velocity_covariance
uint8 reset_counter
uint8 quality
```

#### px4_msgs/VehicleStatus

```
# Vehicle status information
uint64 timestamp
uint8 arming_state
uint8 nav_state
uint8 nav_state_timestamp
uint8 failure_detector_status
# ... additional status fields
```

## Error Handling

### Exception Types

The system defines and handles several exception types:

#### BoundaryError

```python
class BoundaryError(Exception):
    """Raised when waypoint cannot be generated within boundaries."""
    pass
```

#### ParameterError

```python
class ParameterError(Exception):
    """Raised when invalid parameters are provided."""
    pass
```

### Error Recovery

The system implements graceful error recovery:

1. **Algorithm Errors**: Generate safe fallback waypoints
2. **Communication Errors**: Continue with last known good state
3. **Parameter Errors**: Use safe defaults with warnings

## Examples

### Basic Usage

```python
from uav_planning.butterfly import ButterflyExplorer

# Create explorer
explorer = ButterflyExplorer(
    x_min=-10, x_max=10,
    y_min=-5, y_max=5,
    z_min=2, z_max=8,
    alpha=1.5
)

# Generate waypoints
for i in range(10):
    waypoint = explorer.generate_next_waypoint()
    print(f"Waypoint {i}: {waypoint}")
    
    # Simulate reaching waypoint
    explorer.update_position(*waypoint)
```

### ROS 2 Node Usage

```python
import rclpy
from uav_planning.bioinspired_path_generator import BioinspiredPathGenerator

def main():
    rclpy.init()
    node = BioinspiredPathGenerator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Parameter Configuration

```python
# Runtime parameter updates
import rclpy
from rclpy.parameter import Parameter

node = BioinspiredPathGenerator()

# Set parameters
node.set_parameters([
    Parameter('alpha', Parameter.Type.DOUBLE, 1.8),
    Parameter('velocity', Parameter.Type.DOUBLE, 8.0),
    Parameter('x_max', Parameter.Type.DOUBLE, 20.0)
])
```

## Testing the Code

### Unit Test Helpers

```python
from uav_planning.butterfly import ButterflyExplorer
import unittest

class TestButterflyExplorer(unittest.TestCase):
    
    def setUp(self):
        self.explorer = ButterflyExplorer(
            x_min=-5, x_max=5,
            y_min=-5, y_max=5,
            z_min=1, z_max=3
        )
    
    def test_waypoint_generation(self):
        waypoint = self.explorer.generate_next_waypoint()
        self.assertIsInstance(waypoint, tuple)
        self.assertEqual(len(waypoint), 3)
    
    def test_boundary_constraints(self):
        for _ in range(100):
            x, y, z = self.explorer.generate_next_waypoint()
            self.assertGreaterEqual(x, -5)
            self.assertLessEqual(x, 5)
            self.assertGreaterEqual(y, -5)
            self.assertLessEqual(y, 5)
            self.assertGreaterEqual(z, 1)
            self.assertLessEqual(z, 3)
```

### Running Tests

```bash
# Run all tests
cd src/uav_planning/test
python3 test_butterfly_generator.py

# Run specific test
python3 -m pytest test_butterfly_generator.py::TestButterflyExplorer::test_waypoint_generation
```

---

*Code reference last updated: January 5, 2025*
