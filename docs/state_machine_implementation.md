# UAV State Machine Implementation

This document describes the UAV state machine implementation that coordinates all UAV operations including the controller and path generator action.

## Overview

The UAV state machine is the central coordinator for all UAV operations. It manages state transitions, coordinates with the controller and action servers, and ensures safe operation of the UAV.

## State Definitions

The system uses the following states (defined in `UAVState.msg`):

- **IDLE (0)**: UAV is on the ground, disarmed, waiting for commands
- **TAKEOFF (1)**: UAV is taking off to target altitude
- **ACTION_IN_PROGRESS (2)**: UAV is executing a mission (e.g., path generation)
- **HOVER (3)**: UAV is hovering at current position
- **LANDING (4)**: UAV is landing
- **EMERGENCY (5)**: UAV is in emergency mode

## Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   State Machine │    │   UAV Controller│    │ Path Generator  │
│                 │◄──►│                 │    │     Action      │
│  - Coordinates  │    │ - Vehicle Control│    │                 │
│  - State Trans. │    │ - PX4 Interface │    │ - Bio-inspired  │
│  - Safety       │    │ - Trajectory    │    │   Exploration   │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         │                       │                       │
    ┌─────────────────────────────────────────────────────────┐
    │                    ROS 2 Topics/Services                 │
    │                                                         │
    │  - /uav/state                - /uav/waypoint            │
    │  - /uav/start_mission        - /uav/current_pose        │
    │  - /uav/emergency            - /uav/arm                 │
    │  - /uav/land                 - generate_path (action)   │
    └─────────────────────────────────────────────────────────┘
```

## Implementation Details

### 1. UAV State Machine (`uav_state_machine.py`)

The state machine node acts as the central coordinator:

**Key Features:**
- Manages state transitions according to UAV operational requirements
- Provides services for mission control (`/uav/start_mission`, `/uav/emergency`, `/uav/land`)
- Publishes current state information on `/uav/state`
- Coordinates with controller and action servers
- Implements safety checks and emergency handling

**State Transitions:**
```
IDLE → TAKEOFF → ACTION_IN_PROGRESS → HOVER → LANDING → IDLE
  ↑                                     ↓
  └────────────── EMERGENCY ←───────────┘
```

**Services Provided:**
- `/uav/start_mission`: Start a mission (IDLE → TAKEOFF)
- `/uav/emergency`: Trigger emergency mode
- `/uav/land`: Initiate landing sequence

### 2. UAV Controller Integration (`uav_controller.py`)

The controller has been enhanced with state machine awareness:

**New Features:**
- Subscribes to `/uav/state` for state awareness
- Adjusts control behavior based on current state
- Enables/disables control based on state (e.g., disabled in IDLE)
- Provides state-aware trajectory following

**State-Aware Behavior:**
- **IDLE**: Control disabled
- **TAKEOFF/ACTION_IN_PROGRESS/HOVER**: Full control enabled
- **LANDING**: Control enabled for landing trajectory
- **EMERGENCY**: Control enabled for emergency hover

### 3. Path Generator Action Integration (`bioinspired_path_generator_action.py`)

The path generator action server has been enhanced with state integration:

**New Features:**
- Subscribes to `/uav/state` for state awareness
- Rejects goals based on UAV state (e.g., won't accept if already in ACTION_IN_PROGRESS)
- Automatically cancels actions during emergency
- State-aware goal acceptance logic

**Goal Acceptance Logic:**
- **IDLE/HOVER**: Accept goals
- **ACTION_IN_PROGRESS**: Reject (action already running)
- **LANDING/EMERGENCY**: Reject (unsafe states)

## Usage

### Starting the System

1. **Launch the state machine:**
   ```bash
   ros2 run uav_planning uav_state_machine
   ```

2. **Launch the controller:**
   ```bash
   ros2 run uav_planning uav_controller
   ```

3. **Launch the path generator action server:**
   ```bash
   ros2 run uav_planning bioinspired_planner_action
   ```

### Running a Mission

1. **Start the mission:**
   ```bash
   ros2 service call /uav/start_mission std_srvs/srv/SetBool "{data: true}"
   ```

2. **Monitor state:**
   ```bash
   ros2 topic echo /uav/state
   ```

3. **Emergency stop (if needed):**
   ```bash
   ros2 service call /uav/emergency std_srvs/srv/SetBool "{data: true}"
   ```

4. **Land the UAV:**
   ```bash
   ros2 service call /uav/land std_srvs/srv/SetBool "{data: true}"
   ```

### Demo Script

A demo script is provided to test the state machine functionality:

```bash
ros2 run uav_planning state_machine_demo
```

This script demonstrates:
- Starting a mission
- Monitoring state changes
- Initiating landing
- Error handling

## Safety Features

### Emergency Handling
- Any component can trigger emergency mode
- Emergency state immediately cancels all ongoing actions
- UAV transitions to hover mode for safety
- Manual intervention required to clear emergency

### State Validation
- Action servers check state before accepting goals
- Invalid state transitions are prevented
- Automatic state transitions based on vehicle status

### Fault Tolerance
- Handles missing dependencies gracefully
- Continues operation with reduced functionality if PX4 messages unavailable
- Service availability checks with timeouts

## Topics and Services

### Published Topics
- `/uav/state` (uav_interfaces/UAVState): Current state information
- `/uav/waypoint` (geometry_msgs/PoseStamped): Waypoint commands to controller

### Subscribed Topics
- `/fmu/out/vehicle_status` (px4_msgs/VehicleStatus): Vehicle status from PX4
- `/fmu/out/vehicle_odometry` (px4_msgs/VehicleOdometry): Vehicle position

### Services
- `/uav/start_mission` (std_srvs/SetBool): Start/stop mission
- `/uav/emergency` (std_srvs/SetBool): Trigger/clear emergency
- `/uav/land` (std_srvs/SetBool): Initiate landing
- `/uav/arm` (std_srvs/SetBool): Arm/disarm vehicle
- `/uav/set_offboard_mode` (std_srvs/SetBool): Set offboard mode

### Actions
- `generate_path` (uav_interfaces/PathGeneration): Bio-inspired path generation

## Configuration

The state machine can be configured through parameters:

- **Takeoff altitude**: Default 5.0m (configurable in state machine)
- **Path generation bounds**: Configurable in action goal
- **Safety timeouts**: Various timeouts for service calls and state transitions

## Future Enhancements

1. **Parameter Configuration**: Add ROS 2 parameters for runtime configuration
2. **Multiple Mission Types**: Support different types of missions beyond path generation
3. **Recovery Procedures**: Implement automatic recovery from failure states
4. **Logging and Telemetry**: Enhanced logging for mission analysis
5. **Mission Planning**: Integration with mission planning interface

## Troubleshooting

### Common Issues

1. **State machine not accepting goals**
   - Check current state with `ros2 topic echo /uav/state`
   - Ensure UAV is in IDLE or HOVER state

2. **Emergency mode stuck**
   - Clear emergency with `ros2 service call /uav/emergency std_srvs/srv/SetBool "{data: false}"`
   - Check for underlying issues causing emergency

3. **Services not available**
   - Ensure all nodes are running
   - Check network connectivity
   - Verify service names with `ros2 service list`

### Debug Commands

```bash
# Check all available services
ros2 service list | grep uav

# Monitor state changes
ros2 topic echo /uav/state

# Check action server status
ros2 action list

# Monitor waypoint commands
ros2 topic echo /uav/waypoint

# Check vehicle status (if PX4 available)
ros2 topic echo /fmu/out/vehicle_status
```
