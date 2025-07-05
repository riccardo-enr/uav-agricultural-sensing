# UAV Path Planning Launch Files

This directory contains launch files for the UAV agricultural sensing simulation.

## Launch Files

### 1. Complete Simulation (`uav_simulation.launch.py`)

Starts the complete simulation environment including:
- Gazebo simulation
- PX4 SITL (Software In The Loop)
- microXRCE DDS Agent
- Bioinspired path planner

**Usage:**
```bash
ros2 launch uav_planning uav_simulation.launch.py
```

**Parameters:**
- `world`: Gazebo world file (default: `empty.sdf`)
- `vehicle`: PX4 vehicle model (default: `x500`)
- `headless`: Run Gazebo in headless mode (default: `false`)
- `x_min`, `x_max`: X-axis exploration bounds (default: -20.0, 20.0)
- `y_min`, `y_max`: Y-axis exploration bounds (default: -20.0, 20.0)
- `z_min`, `z_max`: Z-axis exploration bounds (default: 5.0, 15.0)
- `velocity`: UAV cruise velocity in m/s (default: 8.0)
- `visit_threshold`: Distance threshold to consider waypoint reached (default: 2.0)

**Example with custom parameters:**
```bash
ros2 launch uav_planning uav_simulation.launch.py \
    world:=agricultural_field.sdf \
    x_min:=-50.0 x_max:=50.0 \
    y_min:=-30.0 y_max:=30.0 \
    z_min:=10.0 z_max:=25.0 \
    velocity:=12.0
```

### 2. Path Planner Only (`path_planner.launch.py`)

Starts only the microXRCE DDS Agent and the bioinspired path planner.
Use this when PX4 and Gazebo are already running.

**Usage:**
```bash
ros2 launch uav_planning path_planner.launch.py
```

**Parameters:**
- Same exploration and velocity parameters as above
- `enable_path_generation`: Enable automatic path generation (default: `true`)

## Manual Setup (Alternative)

If you prefer to start components manually:

1. **Start Gazebo:**
   ```bash
   gz sim -v 4 empty.sdf
   ```

2. **Start PX4 SITL:**
   ```bash
   cd /workspaces/PX4-Autopilot
   make px4_sitl gazebo-classic_iris
   ```

3. **Start microXRCE DDS Agent:**
   ```bash
   MicroXRCEAgent udp4 -p 8888
   ```

4. **Start the path planner:**
   ```bash
   ros2 run uav_planning bioinspired_planner
   ```

## Topics Published

The path planner publishes the following topics:

### Node-specific topics (use `~/` prefix):
- `~/current_waypoint` (geometry_msgs/Point): Current target waypoint
- `~/computation_time` (std_msgs/Float64): Time taken to compute waypoints

### Debug topics:
- `/debug/corners_remaining` (std_msgs/Int32): Number of unvisited corners

### PX4 topics:
- `/fmu/in/trajectory_setpoint` (px4_msgs/TrajectorySetpoint): Position commands
- `/fmu/in/offboard_control_mode` (px4_msgs/OffboardControlMode): Control mode

## Topics Subscribed

- `/fmu/out/vehicle_odometry` (px4_msgs/VehicleOdometry): Vehicle position feedback

## Parameters

All parameters can be set via launch files or command line:

- `x_min`, `x_max`, `y_min`, `y_max`: Exploration area bounds (meters)
- `z_min`, `z_max`: Altitude bounds (meters, positive up)
- `alpha`: Levy flight exponent (default: 1.5)
- `visit_threshold`: Distance to consider waypoint reached (meters)
- `velocity`: Cruise velocity (m/s)
- `waypoint_generation_rate`: Rate for fallback waypoint generation (Hz)
- `trajectory_publish_rate`: Rate for publishing trajectory setpoints (Hz)
- `enable_path_generation`: Enable/disable automatic path generation

## Troubleshooting

1. **"No module named 'px4_msgs'"**: Ensure px4_msgs is installed and sourced
2. **"MicroXRCEAgent: command not found"**: Ensure the agent is built and in PATH
3. **No odometry received**: Check PX4 is running and microXRCE agent is connected
4. **Gazebo doesn't start**: Check GZ_SIM_RESOURCE_PATH includes PX4 models

## Monitoring

To monitor the path planner:

```bash
# Check current waypoint
ros2 topic echo /bioinspired_path_generator/current_waypoint

# Check computation time
ros2 topic echo /bioinspired_path_generator/computation_time

# Check corners remaining
ros2 topic echo /debug/corners_remaining

# Check vehicle odometry
ros2 topic echo /fmu/out/vehicle_odometry
```
