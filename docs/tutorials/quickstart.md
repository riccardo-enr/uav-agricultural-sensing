# Quick Start Tutorial

Get your UAV Agricultural Sensing system up and running in 15 minutes with this step-by-step tutorial.

## Prerequisites

Before starting, ensure you have:
- ✅ Completed the [Installation Guide](../installation.md)
- ✅ Ubuntu 20.04+ with ROS 2 Humble
- ✅ At least 8GB RAM available
- ✅ Working internet connection

## Step 1: Verify Installation

First, let's verify that everything is installed correctly:

```bash
# Check ROS 2 environment
echo $ROS_DISTRO
# Should output: humble

# Check if the package is available
ros2 pkg list | grep uav_planning
# Should output: uav_planning

# Check PX4 installation
ls ~/PX4-Autopilot/build/px4_sitl_default/bin/px4
# Should show the PX4 binary
```

If any of these commands fail, revisit the [Installation Guide](../installation.md).

## Step 2: Source Your Environment

Make sure your ROS 2 environment is properly sourced:

```bash
# Source ROS 2 and your workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Set up PX4 environment
export PX4_DIR=~/PX4-Autopilot
source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
```

**💡 Tip**: Add these lines to your `~/.bashrc` to automatically source them in every terminal.

## Step 3: Run Your First Simulation

### Option A: Complete Simulation (Recommended)

Launch the complete simulation with Gazebo visualization:

```bash
ros2 launch uav_planning uav_simulation.launch.py
```

This command will:
1. Start Gazebo Classic with an empty world
2. Launch PX4 SITL (Software-in-the-Loop)
3. Start the microXRCE DDS Agent
4. Launch the bioinspired path generator
5. Automatically arm the UAV and start path generation

**Expected Output:**
- Gazebo window opens showing a quadcopter
- Terminal shows initialization messages
- UAV automatically arms and begins following waypoints

### Option B: Headless Simulation (For Low-End Systems)

If your system struggles with Gazebo GUI:

```bash
ros2 launch uav_planning uav_simulation.launch.py headless:=true
```

### Option C: Path Planner Only

If you already have PX4 and Gazebo running:

```bash
ros2 launch uav_planning path_planner.launch.py
```

## Step 4: Monitor the System

Open additional terminals to monitor the system:

### Terminal 2: Watch Waypoint Generation

```bash
# Monitor current waypoints
ros2 topic echo /bioinspired_path_generator/current_waypoint
```

### Terminal 3: Check System Status

```bash
# List active nodes
ros2 node list

# Check available topics
ros2 topic list

# Monitor UAV position
ros2 topic echo /fmu/out/vehicle_odometry --field position
```

### Terminal 4: Parameter Monitoring

```bash
# See all parameters
ros2 param list /bioinspired_path_generator

# Get specific parameter values
ros2 param get /bioinspired_path_generator alpha
ros2 param get /bioinspired_path_generator velocity
```

## Step 5: Customize Your First Flight

Let's modify the flight parameters to see the system respond:

### Change Flight Area

```bash
# Make the flight area larger
ros2 param set /bioinspired_path_generator x_max 20.0
ros2 param set /bioinspired_path_generator y_max 20.0
ros2 param set /bioinspired_path_generator z_max 15.0
```

### Adjust Flight Behavior

```bash
# Make the UAV more aggressive (longer flights)
ros2 param set /bioinspired_path_generator alpha 2.0

# Increase flight speed
ros2 param set /bioinspired_path_generator velocity 8.0

# Make waypoint threshold larger (reaches targets easier)
ros2 param set /bioinspired_path_generator visit_threshold 2.0
```

**🎯 Observe**: Watch how the UAV behavior changes in Gazebo as you adjust parameters!

## Step 6: Understanding the Flight Pattern

The UAV follows a **Lévy flight pattern** inspired by butterfly foraging behavior:

- **Short flights**: Local exploration around current area
- **Long flights**: Jumps to new unexplored regions
- **Random directions**: Ensures comprehensive coverage
- **Boundary respect**: Always stays within defined limits

### Key Concepts

1. **Alpha Parameter (α)**: Controls flight aggressiveness
   - `α = 1.2`: More local, conservative flights
   - `α = 1.5`: Balanced behavior (default)
   - `α = 2.0`: More long-range, aggressive flights

2. **Visit Threshold**: Distance to consider waypoint "reached"
   - Smaller values: More precise positioning
   - Larger values: Faster waypoint completion

3. **Velocity**: UAV cruise speed between waypoints

## Step 7: Basic Troubleshooting

### Common Issues and Solutions

#### UAV Not Moving
```bash
# Check if path generation is enabled
ros2 param get /bioinspired_path_generator enable_path_generation

# If false, enable it:
ros2 param set /bioinspired_path_generator enable_path_generation true
```

#### No Gazebo Window
```bash
# Check if you're running headless mode
ros2 launch uav_planning uav_simulation.launch.py headless:=false
```

#### UAV Not Arming
```bash
# Check vehicle status
ros2 topic echo /fmu/out/vehicle_status --field arming_state

# Manually send arm command if needed
ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
  '{command: 400, param1: 1.0, target_system: 1, target_component: 1}'
```

#### Simulation Running Slow
```bash
# Reduce update rates
ros2 param set /bioinspired_path_generator trajectory_publish_rate 5.0
ros2 param set /bioinspired_path_generator waypoint_generation_rate 0.5
```

## Step 8: Save Your Configuration

Once you find settings you like, save them to a file:

```bash
# Create a configuration directory
mkdir -p ~/ros2_ws/src/uav-agricultural-sensing/config

# Save current parameters
ros2 param dump /bioinspired_path_generator > ~/ros2_ws/src/uav-agricultural-sensing/config/my_config.yaml
```

Use your saved configuration:

```bash
ros2 run uav_planning bioinspired_path_generator \
  --ros-args --params-file ~/ros2_ws/src/uav-agricultural-sensing/config/my_config.yaml
```

## Step 9: Stop the Simulation

To cleanly shut down the simulation:

1. **In Gazebo**: Close the Gazebo window
2. **In Terminals**: Press `Ctrl+C` in each terminal running ROS commands
3. **Clean Up**: Wait for all processes to exit cleanly

```bash
# Check for remaining processes
ps aux | grep px4
ps aux | grep gazebo

# Kill any stuck processes if needed
pkill -f px4
pkill -f gazebo
```

## Next Steps

Congratulations! You've successfully run your first UAV agricultural sensing mission. Here's what to explore next:

### 🚀 Immediate Next Steps
1. **Try Different Configurations**: Experiment with the [examples](../examples/)
2. **Learn More Parameters**: Study the [Configuration Guide](../configuration.md)
3. **Understand the Algorithm**: Read the [Algorithm Details](../algorithm_details.md)

### 🔧 Development Path
1. **Write Custom Configurations**: Create specialized configs for different scenarios
2. **Explore the API**: Check out the [API Reference](../api/)
3. **Contribute**: See the [Contributing Guide](../contributing.md)

### 🏭 Production Path
1. **Hardware Integration**: Move from simulation to real UAV
2. **Mission Planning**: Integrate with agriculture-specific missions
3. **Data Collection**: Add sensors and data logging

## Quick Reference Commands

```bash
# Start complete simulation
ros2 launch uav_planning uav_simulation.launch.py

# Start headless simulation
ros2 launch uav_planning uav_simulation.launch.py headless:=true

# Run with custom parameters
ros2 launch uav_planning uav_simulation.launch.py \
  x_min:=-50.0 x_max:=50.0 z_max:=20.0 velocity:=10.0

# Monitor waypoints
ros2 topic echo /bioinspired_path_generator/current_waypoint

# Change parameters on the fly
ros2 param set /bioinspired_path_generator alpha 1.8
ros2 param set /bioinspired_path_generator velocity 12.0

# Save configuration
ros2 param dump /bioinspired_path_generator > my_config.yaml
```

## Troubleshooting Reference

| Problem            | Solution                                          |
| ------------------ | ------------------------------------------------- |
| No UAV movement    | Check `enable_path_generation` parameter          |
| Slow simulation    | Reduce `trajectory_publish_rate`                  |
| UAV outside bounds | Verify `x_min/max`, `y_min/max` parameters        |
| Build errors       | Run `colcon build --packages-select uav_planning` |
| Import errors      | Source ROS 2 and workspace setup files            |

## Getting Help

If you encounter issues:

1. Check the [Troubleshooting Guide](../troubleshooting.md)
2. Review the logs: `ros2 log view`
3. Create an issue on GitHub with:
   - Your exact commands
   - Full error output
   - System information (`lsb_release -a`, `echo $ROS_DISTRO`)

---

**🎉 Congratulations!** You've completed the quick start tutorial. You now have a working UAV agricultural sensing system and understand the basics of how to configure and monitor it.

*Tutorial last updated: January 5, 2025*
