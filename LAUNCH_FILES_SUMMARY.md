# UAV Agricultural Sensing - Launch Files Summary

## Created Launch Files

### 1. `uav_simulation.launch.py` - Complete Simulation
**Purpose**: Starts the complete UAV simulation environment

**Components Started**:
- Gazebo simulation environment
- PX4 SITL (Software In The Loop)
- microXRCE DDS Agent
- Bioinspired path planner

**Usage**:
```bash
ros2 launch uav_planning uav_simulation.launch.py
```

**Key Parameters**:
- `world`: Gazebo world file (default: empty.sdf)
- `vehicle`: PX4 vehicle model (default: x500)
- `x_min/x_max`, `y_min/y_max`: Exploration area bounds
- `z_min/z_max`: Altitude bounds (5.0-15.0m default)
- `velocity`: UAV cruise velocity (8.0 m/s default)
- `visit_threshold`: Waypoint reach distance (2.0m default)

### 2. `path_planner.launch.py` - Path Planner Only
**Purpose**: Starts only the microXRCE DDS Agent and path planner (use when PX4/Gazebo already running)

**Components Started**:
- microXRCE DDS Agent
- Bioinspired path planner

**Usage**:
```bash
ros2 launch uav_planning path_planner.launch.py
```

### 3. `demo.launch.py` - Demo Configurations
**Purpose**: Demonstrates different usage scenarios with predefined parameters

**Modes Available**:
- `planner_only`: Basic path planning (15x15m area)
- `full_simulation`: Complete simulation (25x25m area)
- `agricultural_survey`: Large area survey (200x150m area)

**Usage**:
```bash
# Basic path planning
ros2 launch uav_planning demo.launch.py mode:=planner_only

# Full simulation
ros2 launch uav_planning demo.launch.py mode:=full_simulation

# Agricultural survey
ros2 launch uav_planning demo.launch.py mode:=agricultural_survey
```

## Quick Start Guide

### For Development/Testing:
```bash
# Start only the path planner (when PX4 is already running)
ros2 launch uav_planning path_planner.launch.py

# Or use demo mode
ros2 launch uav_planning demo.launch.py mode:=planner_only
```

### For Complete Simulation:
```bash
# Start everything at once
ros2 launch uav_planning uav_simulation.launch.py

# Or use demo mode
ros2 launch uav_planning demo.launch.py mode:=full_simulation
```

### For Agricultural Surveying:
```bash
# Large area coverage optimized for agricultural applications
ros2 launch uav_planning demo.launch.py mode:=agricultural_survey
```

## Custom Parameters Example:
```bash
ros2 launch uav_planning path_planner.launch.py \
    x_min:=-50.0 x_max:=50.0 \
    y_min:=-30.0 y_max:=30.0 \
    z_min:=10.0 z_max:=25.0 \
    velocity:=12.0 \
    visit_threshold:=3.0
```

## Topics Published:
- `~/current_waypoint`: Current target waypoint
- `~/computation_time`: Waypoint computation time
- `/debug/corners_remaining`: Number of unvisited exploration corners
- `/fmu/in/trajectory_setpoint`: PX4 position commands
- `/fmu/in/offboard_control_mode`: PX4 control mode

## Topics Subscribed:
- `/fmu/out/vehicle_odometry`: Vehicle position feedback (triggers waypoint generation)

## Key Features:
✅ **Odometry-based waypoint generation**: Uses UAV position feedback instead of fixed timeouts
✅ **Configurable exploration area**: Set custom bounds for the survey area
✅ **Bio-inspired Levy flight patterns**: Efficient exploration using butterfly-inspired movements
✅ **PX4 integration**: Direct compatibility with PX4 autopilot
✅ **Fallback mechanisms**: Continues working even without odometry
✅ **Comprehensive logging**: Detailed information about waypoint generation and completion

## Files Created:
- `launch/uav_simulation.launch.py` - Complete simulation launcher
- `launch/path_planner.launch.py` - Path planner only launcher  
- `launch/demo.launch.py` - Demo configurations
- `launch/README.md` - Detailed documentation
- `launch/test_launch.sh` - Launch file validation script
