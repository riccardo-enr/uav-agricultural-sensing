# UAV State Machine

This document describes the UAV State Machine implementation that manages the high-level operational states of the UAV system.

## Overview

The UAV State Machine provides a finite state machine (FSM) that orchestrates UAV operations through well-defined states and transitions. It acts as a high-level coordinator between the UAV controller, path generation actions, and user commands.

## States

The state machine manages the following states:

### 1. IDLE (0)
- **Description**: UAV is on the ground, disarmed, and ready for commands
- **Entry conditions**: System startup, landing complete, emergency disarm
- **Valid transitions**: TAKEOFF
- **Actions**: None (waiting for commands)

### 2. TAKEOFF (1)
- **Description**: UAV is taking off to the target altitude
- **Entry conditions**: Takeoff command from IDLE state
- **Valid transitions**: HOVER, LANDING, EMERGENCY
- **Actions**: Arms vehicle, sets offboard mode, publishes takeoff waypoint
- **Exit condition**: Reaches target altitude (within threshold)

### 3. HOVER (3)
- **Description**: UAV maintains position at current location
- **Entry conditions**: Takeoff complete, action stopped, hover command
- **Valid transitions**: ACTION_IN_PROGRESS, LANDING, EMERGENCY
- **Actions**: Maintains current position, ready for new commands

### 4. ACTION_IN_PROGRESS (2)
- **Description**: UAV is executing a mission (e.g., path generation)
- **Entry conditions**: Start action command from HOVER state
- **Valid transitions**: HOVER, LANDING, EMERGENCY
- **Actions**: Monitors action progress, handles completion
- **Exit condition**: Action completes or is manually stopped

### 5. LANDING (4)
- **Description**: UAV is descending to ground level
- **Entry conditions**: Land command from HOVER or ACTION_IN_PROGRESS
- **Valid transitions**: IDLE, EMERGENCY
- **Actions**: Descends to ground, requests disarm upon completion
- **Exit condition**: Reaches ground level (within threshold)

### 6. EMERGENCY (5)
- **Description**: Emergency state for safety situations
- **Entry conditions**: Emergency stop command, safety violations
- **Valid transitions**: Manual intervention required
- **Actions**: Stops all actions, hovers in place

## State Transitions

```
IDLE → TAKEOFF → HOVER ⇄ ACTION_IN_PROGRESS
  ↑        ↓        ↓
  └─── LANDING ←────┘
          ↓
    EMERGENCY (from any state)
```

## Topics

### Published Topics

- `~/state` (uav_interfaces/UAVState): Current state information including:
  - State ID and description
  - State duration
  - Target position
  - State start time

- `/uav/waypoint` (geometry_msgs/PoseStamped): Target waypoints for the UAV controller

### Subscribed Topics

- `/uav/current_pose` (geometry_msgs/PoseStamped): Current UAV position
- `/uav/armed` (std_msgs/Bool): Armed status from UAV controller

## Services

### Service Servers (provided by state machine)

- `~/takeoff` (std_srvs/Trigger): Request takeoff from IDLE state
- `~/land` (std_srvs/Trigger): Request landing
- `~/hover` (std_srvs/Trigger): Request hover mode
- `~/start_action` (std_srvs/Trigger): Start path generation action
- `~/stop_action` (std_srvs/Trigger): Stop current action

### Service Clients (used by state machine)

- `/uav/arm` (std_srvs/SetBool): Arm/disarm vehicle
- `/uav/set_offboard_mode` (std_srvs/SetBool): Set offboard mode

## Actions

### Action Clients

- `generate_path` (uav_interfaces/PathGeneration): Bio-inspired path generation

## Parameters

- `takeoff_altitude` (double, default: 5.0): Target altitude for takeoff
- `position_threshold` (double, default: 1.0): General position threshold in meters
- `takeoff_threshold` (double, default: 0.5): Altitude threshold for takeoff completion
- `landing_threshold` (double, default: 0.3): Altitude threshold for landing completion

## Usage

### Basic Operation

1. **Start the system:**
   ```bash
   ros2 launch uav_planning uav_system_with_state_machine.launch.py
   ```

2. **Monitor state:**
   ```bash
   ros2 topic echo /uav_state_machine/state
   ```

3. **Control via services:**
   ```bash
   # Takeoff
   ros2 service call /uav_state_machine/takeoff std_srvs/srv/Trigger
   
   # Start action
   ros2 service call /uav_state_machine/start_action std_srvs/srv/Trigger
   
   # Stop action
   ros2 service call /uav_state_machine/stop_action std_srvs/srv/Trigger
   
   # Land
   ros2 service call /uav_state_machine/land std_srvs/srv/Trigger
   ```

### Demo Mode

Run the complete demonstration:
```bash
ros2 launch uav_planning uav_state_machine_demo.launch.py
```

This will automatically cycle through: TAKEOFF → HOVER → ACTION → HOVER → LANDING

## Integration with Existing System

The state machine integrates seamlessly with the existing UAV planning system:

- **UAV Controller**: Handles low-level vehicle control and PX4 communication
- **Path Generator**: Provides bio-inspired path planning as actions
- **UAV Monitor**: Provides system monitoring and status display

## Safety Features

1. **Automatic disarm detection**: Transitions to IDLE if vehicle is unexpectedly disarmed
2. **Emergency stop**: Can transition to EMERGENCY state from any state
3. **Service validation**: Checks current state before allowing transitions
4. **Timeout handling**: Prevents infinite waits in state transitions
5. **Error recovery**: Graceful handling of action failures

## Example Workflow

A typical mission workflow:

1. **System starts in IDLE state**
2. **Request takeoff** → State machine arms vehicle and transitions to TAKEOFF
3. **Takeoff completes** → Automatically transitions to HOVER
4. **Start action** → User requests mission, transitions to ACTION_IN_PROGRESS
5. **Action runs** → State machine monitors path generation progress
6. **Action completes** → Automatically transitions back to HOVER
7. **Request landing** → State machine descends and transitions to LANDING
8. **Landing completes** → Disarms vehicle and returns to IDLE

## Extending the State Machine

To add new states or modify behavior:

1. Add new state to `UAVStateEnum`
2. Implement `_enter_<state>_state()` and `_exit_<state>_state()` methods
3. Add state-specific update logic in `_update_<state>_state()`
4. Update transition logic in service callbacks
5. Add any required topics, services, or actions

## Troubleshooting

### Common Issues

1. **State machine not responding to commands**
   - Check if UAV controller is running
   - Verify service connections
   - Check for error messages in logs

2. **Takeoff fails**
   - Ensure UAV controller can communicate with PX4
   - Check armed status
   - Verify position feedback is available

3. **Action doesn't start**
   - Ensure path generator action server is running
   - Check if UAV is in HOVER state
   - Verify action server connection

### Debug Commands

```bash
# Check state machine status
ros2 topic echo /uav_state_machine/state

# Check available services
ros2 service list | grep uav_state_machine

# Monitor UAV controller status
ros2 topic echo /uav/current_pose
ros2 topic echo /uav/armed

# Check action server
ros2 action list | grep generate_path
```
