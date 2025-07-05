# Separated UAV Architecture

This document describes the separated UAV architecture with two distinct nodes for the UAV agricultural sensing system.

## Architecture Overview

### 1. UAV Controller (`uav_controller.py`)
A continuously running node responsible for:
- **Vehicle Control**: Direct communication with PX4 autopilot
- **Arming/Disarming**: Vehicle safety management
- **Mode Switching**: Offboard/Manual mode control
- **Trajectory Following**: Low-level flight control
- **Status Monitoring**: Real-time vehicle state tracking

#### Topics Published:
- `/uav/current_pose` (PoseStamped) - Current vehicle position
- `/uav/current_velocity` (TwistStamped) - Current vehicle velocity
- `/uav/armed` (Bool) - Vehicle armed status

#### Topics Subscribed:
- `/uav/waypoint` (PoseStamped) - Target waypoint commands
- `/fmu/out/vehicle_odometry` (VehicleOdometry) - PX4 position feedback
- `/fmu/out/vehicle_status` (VehicleStatus) - PX4 status feedback

#### Services Provided:
- `/uav/arm` (SetBool) - Arm/disarm the vehicle
- `/uav/set_offboard_mode` (SetBool) - Set offboard/manual mode

#### PX4 Communication:
- `/fmu/in/trajectory_setpoint` (TrajectorySetpoint) - Position commands
- `/fmu/in/offboard_control_mode` (OffboardControlMode) - Control mode
- `/fmu/in/vehicle_command` (VehicleCommand) - Vehicle commands

### 2. Path Generator Action Server (`path_generator_action.py`)
An action server focused on:
- **Path Planning**: Bio-inspired trajectory generation
- **Mission Management**: High-level mission execution
- **Exploration Logic**: Corner-visiting algorithm
- **Progress Monitoring**: Mission feedback and status

#### Action Interface:
- Action: `PathGeneration` - Bio-inspired path generation mission

#### Topics Published:
- `/uav/waypoint` (PoseStamped) - Generated waypoints to UAV controller

#### Topics Subscribed:
- `/uav/current_pose` (PoseStamped) - Current position from UAV controller
- `/uav/armed` (Bool) - Armed status from UAV controller

#### Service Clients:
- `/uav/arm` (SetBool) - Request arming through UAV controller
- `/uav/set_offboard_mode` (SetBool) - Request mode changes

## Benefits of This Architecture

1. **Separation of Concerns**:
   - UAV Controller: Low-level vehicle control
   - Path Generator: High-level mission planning

2. **Modularity**:
   - Each node can be developed, tested, and debugged independently
   - Easy to replace or upgrade individual components

3. **Reusability**:
   - UAV Controller can be used with different path planning algorithms
   - Path Generator can work with different vehicle controllers

4. **Safety**:
   - UAV Controller always maintains vehicle safety
   - Path Generator cannot directly command unsafe operations

5. **Testability**:
   - Can test path generation logic without real vehicle
   - Can test vehicle control without complex path planning

## Usage

### Launch Both Nodes:
```bash
ros2 launch uav_planning uav_system.launch.py
```

### Launch with Demo Client:
```bash
ros2 launch uav_planning uav_system_demo.launch.py
```

### Manual Control:

1. **Start the system**:
```bash
ros2 run uav_planning uav_controller
ros2 run uav_planning path_generator_action
```

2. **Arm the vehicle** (via service):
```bash
ros2 service call /uav/arm std_srvs/srv/SetBool "{data: true}"
```

3. **Set offboard mode**:
```bash
ros2 service call /uav/set_offboard_mode std_srvs/srv/SetBool "{data: true}"
```

4. **Send a waypoint** (via topic):
```bash
ros2 topic pub --once /uav/waypoint geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 5.0, z: 3.0}}}"
```

5. **Start path generation** (via action):
```bash
ros2 run uav_planning path_generator_client
```

### Monitor Status:
```bash
# Vehicle position
ros2 topic echo /uav/current_pose

# Armed status
ros2 topic echo /uav/armed

# Current waypoint
ros2 topic echo /uav/waypoint
```

## Configuration

Both nodes support standard ROS 2 parameters and can be configured via:
- Launch file parameters
- YAML configuration files
- Command line arguments

## Integration with PX4

The UAV Controller node handles all PX4-specific communication, making it easy to:
- Switch between simulation and real hardware
- Update PX4 interface without affecting path planning
- Add additional safety checks and monitoring
