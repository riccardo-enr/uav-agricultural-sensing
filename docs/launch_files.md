# Launch Files Guide

This guide explains how to use the launch files provided with the UAV Agricultural Sensing system for different deployment scenarios.

## Overview

The system provides several launch files to support different use cases:

- **Complete simulation** with Gazebo, PX4, and path planning
- **Path planner only** for integration with existing systems
- **Demo configurations** for testing and development

## Available Launch Files

### 1. Complete Simulation (`uav_simulation.launch.py`)

**Purpose**: Starts the complete UAV simulation environment including Gazebo, PX4 SITL, microXRCE DDS Agent, and the bioinspired path planner.

**Usage**:
```bash
ros2 launch uav_planning uav_simulation.launch.py
```

**Components Started**:
- Gazebo Classic simulation environment
- PX4 SITL (Software In The Loop) autopilot
- microXRCE DDS Agent for ROS 2 ↔ PX4 communication
- Bioinspired path generator node

**Arguments**:

| Argument          | Type   | Default     | Description                          |
| ----------------- | ------ | ----------- | ------------------------------------ |
| `world`           | string | `empty.sdf` | Gazebo world file to load            |
| `vehicle`         | string | `x500`      | PX4 vehicle model (x500, iris, etc.) |
| `headless`        | bool   | `false`     | Run Gazebo without GUI               |
| `x_min`           | float  | -20.0       | Minimum X boundary (meters)          |
| `x_max`           | float  | 20.0        | Maximum X boundary (meters)          |
| `y_min`           | float  | -20.0       | Minimum Y boundary (meters)          |
| `y_max`           | float  | 20.0        | Maximum Y boundary (meters)          |
| `z_min`           | float  | 5.0         | Minimum altitude (meters)            |
| `z_max`           | float  | 15.0        | Maximum altitude (meters)            |
| `alpha`           | float  | 1.5         | Lévy flight shape parameter          |
| `velocity`        | float  | 8.0         | UAV cruise velocity (m/s)            |
| `visit_threshold` | float  | 2.0         | Waypoint reach distance (meters)     |

**Examples**:

```bash
# Basic simulation with default parameters
ros2 launch uav_planning uav_simulation.launch.py

# Headless simulation for performance
ros2 launch uav_planning uav_simulation.launch.py headless:=true

# Large field simulation
ros2 launch uav_planning uav_simulation.launch.py \
    x_min:=-100.0 x_max:=100.0 \
    y_min:=-75.0 y_max:=75.0 \
    z_min:=10.0 z_max:=25.0 \
    velocity:=12.0

# Agricultural field with custom world
ros2 launch uav_planning uav_simulation.launch.py \
    world:=agricultural_field.sdf \
    alpha:=1.3 \
    velocity:=6.0
```

### 2. Path Planner Only (`path_planner.launch.py`)

**Purpose**: Starts only the microXRCE DDS Agent and bioinspired path planner. Use this when PX4 and Gazebo are already running separately.

**Usage**:
```bash
ros2 launch uav_planning path_planner.launch.py
```

**Components Started**:
- microXRCE DDS Agent
- Bioinspired path generator node

**Arguments**:
All algorithm parameters (same as above, excluding simulation-specific ones like `world`, `vehicle`, `headless`)

**Examples**:

```bash
# Basic path planner
ros2 launch uav_planning path_planner.launch.py

# Custom boundaries
ros2 launch uav_planning path_planner.launch.py \
    x_min:=-50.0 x_max:=50.0 \
    z_min:=8.0 z_max:=20.0

# High-speed coverage
ros2 launch uav_planning path_planner.launch.py \
    alpha:=2.0 \
    velocity:=15.0 \
    visit_threshold:=5.0
```

### 3. Demo Configurations (`demo.launch.py`)

**Purpose**: Demonstrates different usage scenarios with predefined parameters for testing and showcasing the system.

**Usage**:
```bash
ros2 launch uav_planning demo.launch.py mode:=<mode_name>
```

**Available Modes**:

| Mode                  | Area Size | Use Case            | Description                                       |
| --------------------- | --------- | ------------------- | ------------------------------------------------- |
| `planner_only`        | 15×15m    | Development/Testing | Basic path planning for quick iteration           |
| `full_simulation`     | 25×25m    | Complete Demo       | Full simulation with moderate area                |
| `agricultural_survey` | 200×150m  | Production          | Large area coverage for agricultural applications |

**Examples**:
```bash
# Basic path planning for development
ros2 launch uav_planning demo.launch.py mode:=planner_only

# Complete simulation demo
ros2 launch uav_planning demo.launch.py mode:=full_simulation

# Large-scale agricultural survey
ros2 launch uav_planning demo.launch.py mode:=agricultural_survey
```

## Separated Architecture Launch Files

The new separated architecture provides additional launch files for modular operation:

### UAV System Launch (`uav_system.launch.py`)

**Purpose**: Launches both the UAV controller and path generator action server for the separated architecture.

**Usage**:
```bash
ros2 launch uav_planning uav_system.launch.py
```

**Components Started**:
- UAV Controller node (continuous vehicle control)
- Path Generator Action Server (on-demand mission planning)

**Arguments**:

| Argument       | Type   | Default | Description             |
| -------------- | ------ | ------- | ----------------------- |
| `namespace`    | string | ""      | Namespace for the nodes |
| `use_sim_time` | bool   | false   | Use simulation time     |

### UAV System Demo (`uav_system_demo.launch.py`)

**Purpose**: Launches the separated architecture with an automatic demo client.

**Usage**:
```bash
ros2 launch uav_planning uav_system_demo.launch.py
```

**Components Started**:
- UAV Controller node
- Path Generator Action Server
- Demo client (starts automatically after 3 seconds)

**Arguments**:

| Argument          | Type   | Default | Description                     |
| ----------------- | ------ | ------- | ------------------------------- |
| `namespace`       | string | ""      | Namespace for the nodes         |
| `use_sim_time`    | bool   | false   | Use simulation time             |
| `auto_start_demo` | bool   | true    | Automatically start demo client |

The demo client uses these default parameters:
- Exploration area: [-10, 10] x [-10, 10] x [1, 5] meters
- Alpha: 2.0
- Visit threshold: 2.0 meters
- Max waypoints: 20
- Exploration time: 120 seconds
- Auto arm: false (for safety)

## Manual Component Startup

For development and debugging, you can start components individually:

### 1. UAV Controller
```bash
ros2 run uav_planning uav_controller
```

### 2. Path Generator Action Server
```bash
ros2 run uav_planning path_generator_action
```

### 3. Architecture Test
```bash
ros2 run uav_planning architecture_test
```

### 4. System Monitor
```bash
ros2 run uav_planning uav_monitor
```

### 5. Demo Client
```bash
ros2 run uav_planning path_generator_client
```

## Legacy Launch Files

The following launch files support the original monolithic architecture:

### 1. Complete Simulation (`uav_simulation.launch.py`)

**Purpose**: Starts the complete UAV simulation environment including Gazebo, PX4 SITL, microXRCE DDS Agent, and the bioinspired path planner.

**Usage**:
```bash
ros2 launch uav_planning uav_simulation.launch.py
```

**Components Started**:
- Gazebo Classic simulation environment
- PX4 SITL (Software In The Loop) autopilot
- microXRCE DDS Agent for ROS 2 ↔ PX4 communication
- Bioinspired path generator node

**Arguments**:

| Argument          | Type   | Default     | Description                          |
| ----------------- | ------ | ----------- | ------------------------------------ |
| `world`           | string | `empty.sdf` | Gazebo world file to load            |
| `vehicle`         | string | `x500`      | PX4 vehicle model (x500, iris, etc.) |
| `headless`        | bool   | `false`     | Run Gazebo without GUI               |
| `x_min`           | float  | -20.0       | Minimum X boundary (meters)          |
| `x_max`           | float  | 20.0        | Maximum X boundary (meters)          |
| `y_min`           | float  | -20.0       | Minimum Y boundary (meters)          |
| `y_max`           | float  | 20.0        | Maximum Y boundary (meters)          |
| `z_min`           | float  | 5.0         | Minimum altitude (meters)            |
| `z_max`           | float  | 15.0        | Maximum altitude (meters)            |
| `alpha`           | float  | 1.5         | Lévy flight shape parameter          |
| `velocity`        | float  | 8.0         | UAV cruise velocity (m/s)            |
| `visit_threshold` | float  | 2.0         | Waypoint reach distance (meters)     |

**Examples**:

```bash
# Basic simulation with default parameters
ros2 launch uav_planning uav_simulation.launch.py

# Headless simulation for performance
ros2 launch uav_planning uav_simulation.launch.py headless:=true

# Large field simulation
ros2 launch uav_planning uav_simulation.launch.py \
    x_min:=-100.0 x_max:=100.0 \
    y_min:=-75.0 y_max:=75.0 \
    z_min:=10.0 z_max:=25.0 \
    velocity:=12.0

# Agricultural field with custom world
ros2 launch uav_planning uav_simulation.launch.py \
    world:=agricultural_field.sdf \
    alpha:=1.3 \
    velocity:=6.0
```

### 2. Path Planner Only (`path_planner.launch.py`)

**Purpose**: Starts only the microXRCE DDS Agent and bioinspired path planner. Use this when PX4 and Gazebo are already running separately.

**Usage**:
```bash
ros2 launch uav_planning path_planner.launch.py
```

**Components Started**:
- microXRCE DDS Agent
- Bioinspired path generator node

**Arguments**:
All algorithm parameters (same as above, excluding simulation-specific ones like `world`, `vehicle`, `headless`)

**Examples**:

```bash
# Basic path planner
ros2 launch uav_planning path_planner.launch.py

# Custom boundaries
ros2 launch uav_planning path_planner.launch.py \
    x_min:=-50.0 x_max:=50.0 \
    z_min:=8.0 z_max:=20.0

# High-speed coverage
ros2 launch uav_planning path_planner.launch.py \
    alpha:=2.0 \
    velocity:=15.0 \
    visit_threshold:=5.0
```

### 3. Demo Configurations (`demo.launch.py`)

**Purpose**: Demonstrates different usage scenarios with predefined parameters for testing and showcasing the system.

**Usage**:
```bash
ros2 launch uav_planning demo.launch.py mode:=<mode_name>
```

**Available Modes**:

| Mode                  | Area Size | Use Case            | Description                                       |
| --------------------- | --------- | ------------------- | ------------------------------------------------- |
| `planner_only`        | 15×15m    | Development/Testing | Basic path planning for quick iteration           |
| `full_simulation`     | 25×25m    | Complete Demo       | Full simulation with moderate area                |
| `agricultural_survey` | 200×150m  | Production          | Large area coverage for agricultural applications |

**Examples**:
```bash
# Basic path planning for development
ros2 launch uav_planning demo.launch.py mode:=planner_only

# Complete simulation demo
ros2 launch uav_planning demo.launch.py mode:=full_simulation

# Large-scale agricultural survey
ros2 launch uav_planning demo.launch.py mode:=agricultural_survey
```

## ROS 2 Topics Reference

### Published Topics

| Topic                           | Type                           | Description                                     |
| ------------------------------- | ------------------------------ | ----------------------------------------------- |
| `~/current_waypoint`            | `geometry_msgs/PointStamped`   | Current target waypoint being pursued           |
| `~/computation_time`            | `std_msgs/Float64`             | Time taken to compute the current waypoint (ms) |
| `/debug/corners_remaining`      | `std_msgs/Int32`               | Number of unvisited exploration corners         |
| `/fmu/in/trajectory_setpoint`   | `px4_msgs/TrajectorySetpoint`  | PX4 position commands                           |
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | PX4 control mode settings                       |

### Subscribed Topics

| Topic                       | Type                       | Description                                              |
| --------------------------- | -------------------------- | -------------------------------------------------------- |
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | Vehicle position feedback (triggers waypoint generation) |

**Note**: All topics with `~` prefix use the node's namespace. For example, if the node runs as `/bioinspired_path_generator`, the full topic would be `/bioinspired_path_generator/current_waypoint`.

## Key Features Summary

✅ **Odometry-based waypoint generation**: Uses UAV position feedback instead of fixed timeouts  
✅ **Configurable exploration area**: Set custom bounds for the survey area  
✅ **Bio-inspired Lévy flight patterns**: Efficient exploration using butterfly-inspired movements  
✅ **PX4 integration**: Direct compatibility with PX4 autopilot  
✅ **Fallback mechanisms**: Continues working even without odometry  
✅ **Comprehensive logging**: Detailed information about waypoint generation and completion  

## Launch File Customization

### Using Parameter Files

Create custom parameter files for repeatable configurations:

**my_config.yaml**:
```yaml
/**:
  ros__parameters:
    x_min: -25.0
    x_max: 25.0
    y_min: -15.0
    y_max: 15.0
    z_min: 8.0
    z_max: 12.0
    alpha: 1.4
    velocity: 7.0
    visit_threshold: 2.5
```

**Usage**:
```bash
ros2 launch uav_planning uav_simulation.launch.py \
    --ros-args --params-file my_config.yaml
```

### Environment-Specific Configurations

#### Development Environment
```bash
# Fast iteration with small area and high update rates
ros2 launch uav_planning uav_simulation.launch.py \
    x_min:=-10.0 x_max:=10.0 \
    y_min:=-10.0 y_max:=10.0 \
    headless:=true \
    velocity:=5.0
```

#### Production Environment
```bash
# Conservative settings for real hardware
ros2 launch uav_planning path_planner.launch.py \
    alpha:=1.3 \
    velocity:=6.0 \
    visit_threshold:=3.0
```

#### Research Environment
```bash
# Detailed logging and analysis
export ROS_LOG_LEVEL=DEBUG
ros2 launch uav_planning uav_simulation.launch.py \
    alpha:=1.5 \
    velocity:=8.0 \
    # Additional logging and recording setup
```

## Integration Workflows

### Workflow 1: Complete Development Cycle

```bash
# 1. Start complete simulation for development
ros2 launch uav_planning uav_simulation.launch.py headless:=true

# 2. Monitor in separate terminals
ros2 topic echo /bioinspired_path_generator/current_waypoint
ros2 topic echo /fmu/out/vehicle_odometry

# 3. Adjust parameters dynamically
ros2 param set /bioinspired_path_generator alpha 1.8
ros2 param set /bioinspired_path_generator velocity 10.0

# 4. Save working configuration
ros2 param dump /bioinspired_path_generator > my_working_config.yaml
```

### Workflow 2: Hardware Integration

```bash
# 1. Start PX4 on real hardware
# (Hardware-specific startup commands)

# 2. Start path planner only
ros2 launch uav_planning path_planner.launch.py \
    alpha:=1.3 \
    velocity:=6.0 \
    visit_threshold:=3.0

# 3. Monitor system health
ros2 topic hz /fmu/in/trajectory_setpoint
ros2 topic echo /fmu/out/vehicle_status
```

### Workflow 3: Multi-Vehicle Operations

```bash
# Vehicle 1 (namespace: uav1)
ROS_NAMESPACE=uav1 ros2 launch uav_planning path_planner.launch.py \
    x_min:=-50.0 x_max:=0.0

# Vehicle 2 (namespace: uav2)  
ROS_NAMESPACE=uav2 ros2 launch uav_planning path_planner.launch.py \
    x_min:=0.0 x_max:=50.0
```

## Troubleshooting Launch Issues

### Common Problems

#### Launch File Not Found
```bash
# Check package installation
ros2 pkg list | grep uav_planning

# Rebuild if necessary
colcon build --packages-select uav_planning
source install/setup.bash
```

#### Gazebo Fails to Start
```bash
# Try headless mode
ros2 launch uav_planning uav_simulation.launch.py headless:=true

# Check Gazebo installation
gazebo --version
```

#### PX4 Connection Issues
```bash
# Check if PX4 process is running
ps aux | grep px4

# Check microXRCE DDS Agent
ps aux | grep micro-xrce

# Manually start DDS agent if needed
micro-xrce-dds-agent udp4 -p 8888
```

#### Parameter Override Issues
```bash
# Check parameter loading
ros2 param list /bioinspired_path_generator

# Verify parameter values
ros2 param get /bioinspired_path_generator alpha

# Reset to defaults if needed
ros2 param set /bioinspired_path_generator alpha 1.5
```

### Debug Mode

Enable detailed logging for troubleshooting:

```bash
export ROS_LOG_LEVEL=DEBUG
ros2 launch uav_planning uav_simulation.launch.py
```

### Clean Startup

If experiencing persistent issues:

```bash
# Kill all related processes
pkill -f gazebo
pkill -f px4
pkill -f micro-xrce

# Clear any temporary files
rm -rf /tmp/px4*
rm -rf ~/.gazebo/models/.database_cache

# Restart with clean state
ros2 launch uav_planning uav_simulation.launch.py
```

## Performance Optimization

### Simulation Performance

```bash
# Reduce graphical load
ros2 launch uav_planning uav_simulation.launch.py \
    headless:=true

# Reduce update rates
ros2 launch uav_planning uav_simulation.launch.py \
    # Lower trajectory publish rate via parameter file
```

### Network Performance

```bash
# Set DDS domain for isolation
export ROS_DOMAIN_ID=42
ros2 launch uav_planning uav_simulation.launch.py

# Use faster DDS implementation
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch uav_planning uav_simulation.launch.py
```

## Advanced Usage

### Custom Launch Files

Create your own launch files based on the provided templates:

```python
# my_custom_launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('uav_planning'), 
                '/launch/uav_simulation.launch.py'
            ]),
            launch_arguments={
                'x_min': '-30.0',
                'x_max': '30.0',
                'alpha': '1.6',
                'velocity': '9.0'
            }.items()
        )
    ])
```

### Automated Testing

```bash
# Automated test with timeout
timeout 60s ros2 launch uav_planning uav_simulation.launch.py headless:=true

# Check if test passed (based on topics being published)
ros2 topic hz /bioinspired_path_generator/current_waypoint --window 10
```

---

*Launch files guide last updated: January 5, 2025*
