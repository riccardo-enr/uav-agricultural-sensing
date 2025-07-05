# Quick Start Guide - Separated UAV Architecture

## Overview

The UAV system has been successfully split into two main components:

### 🎯 **UAV Controller** (`uav_controller`)
- **Purpose**: Low-level vehicle control and safety
- **Responsibilities**: PX4 communication, arming, mode switching, trajectory following
- **Runs**: Continuously in the background

### 🧠 **Path Generator Action** (`path_generator_action`)
- **Purpose**: High-level mission planning and execution
- **Responsibilities**: Bio-inspired path generation, mission management
- **Runs**: On-demand via action interface

## Quick Testing

### 1. Test the Architecture
```bash
# Terminal 1: Start UAV controller
ros2 run uav_planning uav_controller

# Terminal 2: Run architecture test
ros2 run uav_planning architecture_test
```

### 2. Monitor System Status
```bash
# Terminal 3: Start real-time monitor
ros2 run uav_planning uav_monitor
```

### 3. Full System Demo
```bash
# All-in-one launch (includes demo mission)
ros2 launch uav_planning uav_system_demo.launch.py
```

## Manual Operation

### Start Individual Components
```bash
# Terminal 1: UAV Controller
ros2 run uav_planning uav_controller

# Terminal 2: Path Generator Action Server
ros2 run uav_planning path_generator_action

# Terminal 3: Optional monitoring
ros2 run uav_planning uav_monitor
```

### Control via Services
```bash
# Arm the vehicle
ros2 service call /uav/arm std_srvs/srv/SetBool "{data: true}"

# Set offboard mode
ros2 service call /uav/set_offboard_mode std_srvs/srv/SetBool "{data: true}"
```

### Send Manual Waypoints
```bash
# Send a waypoint
ros2 topic pub --once /uav/waypoint geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0, z: 2.0}}}"
```

### Start Bio-inspired Mission
```bash
# Run the demo client
ros2 run uav_planning path_generator_client
```

## Key Benefits

✅ **Modularity**: Each component has a single responsibility  
✅ **Safety**: UAV controller maintains vehicle safety independently  
✅ **Testability**: Can test path planning without real vehicle  
✅ **Reusability**: Components can be used with different systems  
✅ **Maintainability**: Easier to debug and upgrade individual parts  

## System Status Topics

Monitor these topics to understand system state:

```bash
# Current position
ros2 topic echo /uav/current_pose

# Current velocity  
ros2 topic echo /uav/current_velocity

# Armed status
ros2 topic echo /uav/armed

# Active waypoint
ros2 topic echo /uav/waypoint
```

## File Structure

```
uav_planning/
├── uav_controller.py           # Low-level UAV control
├── path_generator_action.py    # High-level path planning
├── architecture_test.py        # System verification
├── uav_monitor.py             # Real-time monitoring
└── launch/
    ├── uav_system.launch.py       # Basic system launch
    └── uav_system_demo.launch.py  # Demo with client
```

## Next Steps

1. **Test the separated architecture** with the test scripts
2. **Integrate with your specific UAV hardware** by updating PX4 topics
3. **Customize path planning parameters** in the action client
4. **Add additional safety features** to the UAV controller
5. **Extend monitoring capabilities** as needed

The separated architecture provides a solid foundation for complex UAV missions while maintaining safety and modularity! 🚁
