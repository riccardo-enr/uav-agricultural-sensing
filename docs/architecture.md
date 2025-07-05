# Architecture Guide

This document provides a detailed overview of the UAV Agricultural Sensing system architecture, explaining how components interact and the design decisions behind the implementation.

## System Overview

The UAV Agricultural Sensing system is built on a modular architecture that separates concerns between path planning algorithms, ROS 2 integration, and UAV control systems.

```
┌─────────────────────────────────────────────────────────────────┐
│                    UAV Agricultural Sensing                     │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐  ┌────────────────┐  ┌─────────────────┐   │
│  │   User Layer    │  │  Config Layer  │  │   Debug Layer   │   │
│  │                 │  │                │  │                 │   │
│  │ • Launch Files  │  │ • Parameters   │  │ • Visualization │   │
│  │ • CLI Tools     │  │ • YAML Files   │  │ • Logging       │   │
│  └─────────────────┘  └────────────────┘  └─────────────────┘   │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │                    ROS 2 Layer                             │  │
│  │                                                            │  │
│  │  ┌───────────────────────────────────────────────────────┐ │  │
│  │  │         BioinspiredPathGenerator Node               │ │  │
│  │  │                                                     │ │  │
│  │  │  • Parameter Management                             │ │  │
│  │  │  • Topic Publishing/Subscribing                     │ │  │
│  │  │  • Timer Management                                 │ │  │
│  │  │  • State Management                                 │ │  │
│  │  └───────────────────────────────────────────────────────┘ │  │
│  └─────────────────────────────────────────────────────────────┘  │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │                   Algorithm Layer                          │  │
│  │                                                            │  │
│  │  ┌─────────────────────┐  ┌─────────────────────────────┐  │  │
│  │  │   ButterflyExplorer │  │      Support Classes       │  │  │
│  │  │                     │  │                             │  │  │
│  │  │ • Lévy Flight Gen   │  │ • Polygon (boundaries)     │  │  │
│  │  │ • Waypoint Logic    │  │ • Utility Functions        │  │  │
│  │  │ • State Tracking    │  │ • Math Helpers             │  │  │
│  │  └─────────────────────┘  └─────────────────────────────┘  │  │
│  └─────────────────────────────────────────────────────────────┘  │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │                 Communication Layer                        │  │
│  │                                                            │  │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────┐ │  │
│  │  │   PX4 Direct    │  │   MAVROS Alt    │  │   Debug     │ │  │
│  │  │                 │  │                 │  │             │ │  │
│  │  │ • px4_msgs      │  │ • geometry_msgs │  │ • std_msgs  │ │  │
│  │  │ • TrajectorySet │  │ • PoseStamped   │  │ • Float64   │ │  │
│  │  │ • VehicleCmd    │  │ • TwistStamped  │  │ • Point     │ │  │
│  │  └─────────────────┘  └─────────────────┘  └─────────────┘ │  │
│  └─────────────────────────────────────────────────────────────┘  │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │                    Hardware Layer                          │  │
│  │                                                            │  │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────┐ │  │
│  │  │   Simulation    │  │   Real Hardware │  │   Testing   │ │  │
│  │  │                 │  │                 │  │             │ │  │
│  │  │ • Gazebo+PX4    │  │ • Physical UAV  │  │ • Unit Tests│ │  │
│  │  │ • SITL          │  │ • Flight Stack  │  │ • Mocking   │ │  │
│  │  └─────────────────┘  └─────────────────┘  └─────────────┘ │  │
│  └─────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

## Core Components

### 1. BioinspiredPathGenerator Node

The main ROS 2 node that orchestrates the entire system.

**Responsibilities:**
- Parameter management and validation
- ROS 2 topic publishing and subscribing
- Timer-based execution control
- State management and monitoring
- Integration with PX4 autopilot

**Key Features:**
- **Modular Design**: Cleanly separated algorithm from ROS integration
- **Robust Error Handling**: Graceful degradation when components fail
- **Hot Reconfiguration**: Dynamic parameter updates without restart
- **Multi-Platform Support**: Works with simulation and real hardware

### 2. ButterflyExplorer Algorithm

Pure Python implementation of the bio-inspired path planning algorithm.

**Core Algorithm:**
```python
class ButterflyExplorer:
    def __init__(self, boundaries, alpha, visit_threshold):
        self.boundaries = boundaries
        self.alpha = alpha  # Lévy flight parameter
        self.visit_threshold = visit_threshold
        self.current_position = None
        self.unvisited_corners = self._initialize_corners()
    
    def generate_next_waypoint(self):
        # 1. Generate Lévy flight step
        step_length = self._levy_flight_step()
        direction = self._random_direction()
        
        # 2. Calculate candidate position
        candidate = self.current_position + step_length * direction
        
        # 3. Apply boundary constraints
        waypoint = self._apply_boundaries(candidate)
        
        # 4. Update exploration state
        self._update_exploration_state(waypoint)
        
        return waypoint
```

**Design Principles:**
- **Stateless Operations**: Each waypoint generation is independent
- **Deterministic Behavior**: Same parameters produce similar exploration patterns
- **Boundary Respect**: Hard constraints ensure operational safety
- **Exploration Tracking**: Maintains state for coverage optimization

### 3. Communication Layer

Handles all external communication using ROS 2 messaging.

**PX4 Integration (Primary):**
- Direct communication via `px4_msgs`
- High-frequency trajectory setpoints
- Vehicle status monitoring
- Automatic arming and mode switching

**MAVROS Integration (Alternative):**
- Standard ROS ecosystem compatibility
- Broader autopilot support
- Easier integration with existing systems

**Debug Interface:**
- Real-time parameter monitoring
- Exploration statistics
- Performance metrics

## Data Flow Architecture

### 1. Initialization Flow

```
System Start
     ↓
Load Parameters
     ↓
Initialize ButterflyExplorer
     ↓
Setup ROS Publishers/Subscribers
     ↓
Start Timer Callbacks
     ↓
Generate Initial Waypoint
     ↓
System Ready
```

### 2. Runtime Data Flow

```
Timer Tick (1-10 Hz)
     ↓
Check Vehicle Position
     ↓
Waypoint Reached? ──No──┐
     ↓ Yes              │
Generate New Waypoint   │
     ↓                  │
Publish Trajectory ←────┘
     ↓
Update Debug Info
     ↓
Wait for Next Tick
```

### 3. State Management

The system maintains several state variables:

**Position State:**
- `current_position`: UAV location (x, y, z)
- `current_waypoint`: Target location
- `waypoint_reached`: Boolean flag

**Vehicle State:**
- `vehicle_armed`: Armed status
- `offboard_mode_active`: Control mode
- `nav_state`: Navigation state code

**Algorithm State:**
- `unvisited_corners`: Exploration tracking
- `total_steps`: Step counter
- `exploration_stats`: Coverage metrics

## Message Flow

### Input Messages

| Topic                       | Message Type      | Purpose           |
| --------------------------- | ----------------- | ----------------- |
| `/fmu/out/vehicle_odometry` | `VehicleOdometry` | Position feedback |
| `/fmu/out/vehicle_status`   | `VehicleStatus`   | Vehicle state     |

### Output Messages

| Topic                           | Message Type          | Purpose             |
| ------------------------------- | --------------------- | ------------------- |
| `/fmu/in/trajectory_setpoint`   | `TrajectorySetpoint`  | Waypoint commands   |
| `/fmu/in/offboard_control_mode` | `OffboardControlMode` | Control mode        |
| `/fmu/in/vehicle_command`       | `VehicleCommand`      | Arm/disarm commands |

### Debug Messages

| Topic                      | Message Type | Purpose              |
| -------------------------- | ------------ | -------------------- |
| `~/current_waypoint`       | `Point`      | Current target       |
| `~/computation_time`       | `Float64`    | Algorithm timing     |
| `/debug/corners_remaining` | `Int32`      | Exploration progress |

## Quality of Service (QoS) Design

The system uses carefully tuned QoS profiles for different message types:

**Real-time Control Messages:**
```python
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
```

**Debug Messages:**
```python
debug_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

## Error Handling Strategy

### Graceful Degradation

The system is designed to continue operation even when subsystems fail:

1. **PX4 Messages Unavailable**: Falls back to MAVROS or debug-only mode
2. **Odometry Loss**: Uses fallback waypoint generation
3. **Parameter Errors**: Uses safe defaults with warnings
4. **Algorithm Errors**: Generates safe waypoints within boundaries

### Error Recovery

```python
try:
    waypoint = self.explorer.generate_next_waypoint()
except Exception as e:
    self.get_logger().error(f"Algorithm error: {e}")
    # Generate safe fallback waypoint
    waypoint = self._generate_safe_waypoint()
```

## Performance Considerations

### Computational Complexity

- **Waypoint Generation**: O(1) - constant time
- **Boundary Checking**: O(1) - simple clipping
- **State Updates**: O(1) - hash table operations

### Memory Usage

- **Algorithm State**: ~1KB per 1000 waypoints
- **ROS Overhead**: ~10MB baseline
- **Message Buffers**: Configurable, typically ~1MB

### Real-time Performance

- **Target Latency**: <10ms for waypoint generation
- **Update Rate**: 1-10 Hz waypoint generation
- **Publish Rate**: Up to 50 Hz trajectory updates

## Extension Points

The architecture provides several extension points for customization:

### 1. Custom Algorithms

Replace `ButterflyExplorer` with custom implementations:

```python
class CustomExplorer:
    def generate_next_waypoint(self):
        # Custom algorithm implementation
        pass

# In the ROS node:
self.explorer = CustomExplorer(parameters)
```

### 2. Additional Sensors

Add new sensor inputs by extending the subscriber callbacks:

```python
def lidar_callback(self, msg):
    # Process LiDAR data for obstacle avoidance
    self.explorer.update_obstacles(obstacles)
```

### 3. Mission Planning

Integrate with higher-level mission planners:

```python
def mission_callback(self, msg):
    # Update boundaries based on mission requirements
    self.explorer.update_boundaries(new_boundaries)
```

## Testing Architecture

### Unit Testing

- **Algorithm Tests**: Pure Python testing of core logic
- **ROS Tests**: Mock-based testing of node functionality
- **Integration Tests**: End-to-end simulation testing

### Simulation Testing

- **Gazebo Integration**: Full physics simulation
- **SITL Testing**: Software-in-the-loop validation
- **Hardware-in-the-loop**: Semi-real testing

### Validation Methods

- **Coverage Analysis**: Measure exploration completeness
- **Path Efficiency**: Analyze flight patterns
- **Safety Validation**: Boundary compliance testing

## Security Considerations

### Input Validation

- All parameters are validated on load
- Boundary checks prevent invalid waypoints
- Rate limiting prevents command flooding

### Safe Defaults

- Conservative flight parameters
- Automatic boundary enforcement
- Emergency stop capabilities

---

*Architecture guide last updated: January 5, 2025*
