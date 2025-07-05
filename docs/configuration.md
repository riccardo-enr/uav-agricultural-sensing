# Configuration Guide

This guide covers all configuration options available in the UAV Agricultural Sensing system, from basic parameter tuning to advanced customization.

## Overview

The system provides extensive configuration through ROS 2 parameters, launch file arguments, and configuration files. Parameters can be set at runtime or through launch files for persistent configurations.

## Core Parameters

### Boundary Configuration

These parameters define the operational area for the UAV:

| Parameter | Type   | Default | Range   | Description                 |
| --------- | ------ | ------- | ------- | --------------------------- |
| `x_min`   | double | -10.0   | Any     | Minimum X boundary (meters) |
| `x_max`   | double | 10.0    | Any     | Maximum X boundary (meters) |
| `y_min`   | double | -10.0   | Any     | Minimum Y boundary (meters) |
| `y_max`   | double | 10.0    | Any     | Maximum Y boundary (meters) |
| `z_min`   | double | 2.0     | > 0     | Minimum altitude (meters)   |
| `z_max`   | double | 8.0     | > z_min | Maximum altitude (meters)   |

**Example Usage:**
```bash
ros2 run uav_planning bioinspired_path_generator \
  --ros-args \
  -p x_min:=-50.0 \
  -p x_max:=50.0 \
  -p y_min:=-30.0 \
  -p y_max:=30.0 \
  -p z_min:=5.0 \
  -p z_max:=15.0
```

### Algorithm Parameters

Control the behavior of the Lévy flight algorithm:

| Parameter         | Type   | Default | Range         | Description                                    |
| ----------------- | ------ | ------- | ------------- | ---------------------------------------------- |
| `alpha`           | double | 1.5     | 1.0 < α ≤ 3.0 | Lévy flight shape parameter                    |
| `visit_threshold` | double | 1.2     | > 0           | Distance to consider waypoint reached (meters) |
| `velocity`        | double | 5.0     | > 0           | UAV cruise velocity (m/s)                      |

**Alpha Parameter Guide:**
- `α = 1.2`: Conservative search, more local exploration
- `α = 1.5`: Balanced exploration/exploitation (recommended)
- `α = 1.8`: More aggressive, longer flights
- `α = 2.0`: Maximum long-range behavior

### Timing Parameters

Control the update rates and timing:

| Parameter                  | Type   | Default | Range | Description                          |
| -------------------------- | ------ | ------- | ----- | ------------------------------------ |
| `waypoint_generation_rate` | double | 1.0     | > 0   | Waypoint generation frequency (Hz)   |
| `trajectory_publish_rate`  | double | 10.0    | > 0   | Trajectory publishing frequency (Hz) |

### System Control

Enable/disable system features:

| Parameter                | Type | Default | Description                              |
| ------------------------ | ---- | ------- | ---------------------------------------- |
| `enable_path_generation` | bool | true    | Enable/disable automatic path generation |

## Launch File Configuration

### Complete Simulation Launch

The `uav_simulation.launch.py` file supports these arguments:

```bash
ros2 launch uav_planning uav_simulation.launch.py \
    world:=agricultural_field.sdf \
    vehicle:=x500 \
    headless:=false \
    x_min:=-100.0 x_max:=100.0 \
    y_min:=-60.0 y_max:=60.0 \
    z_min:=10.0 z_max:=25.0 \
    velocity:=12.0 \
    visit_threshold:=3.0 \
    alpha:=1.8
```

#### Launch Arguments

| Argument                 | Type   | Default   | Description            |
| ------------------------ | ------ | --------- | ---------------------- |
| `world`                  | string | empty.sdf | Gazebo world file      |
| `vehicle`                | string | x500      | PX4 vehicle model      |
| `headless`               | bool   | false     | Run Gazebo without GUI |
| All algorithm parameters |        |           | Same as above          |

### Path Planner Only Launch

The `path_planner.launch.py` file is used when PX4 and Gazebo are already running:

```bash
ros2 launch uav_planning path_planner.launch.py \
    x_min:=-20.0 x_max:=20.0 \
    velocity:=8.0
```

## Configuration Files

### Parameter Files

Create YAML parameter files for persistent configurations:

**config/agricultural_field.yaml:**
```yaml
/**:
  ros__parameters:
    # Large agricultural field configuration
    x_min: -100.0
    x_max: 100.0
    y_min: -80.0
    y_max: 80.0
    z_min: 10.0
    z_max: 20.0
    
    # Conservative exploration for detailed coverage
    alpha: 1.3
    visit_threshold: 2.5
    velocity: 8.0
    
    # Higher update rates for precision
    waypoint_generation_rate: 0.5
    trajectory_publish_rate: 20.0
```

**config/greenhouse.yaml:**
```yaml
/**:
  ros__parameters:
    # Greenhouse configuration - smaller area
    x_min: -15.0
    x_max: 15.0
    y_min: -10.0
    y_max: 10.0
    z_min: 2.0
    z_max: 5.0
    
    # More aggressive exploration for quick coverage
    alpha: 2.0
    visit_threshold: 1.0
    velocity: 3.0
    
    # Standard update rates
    waypoint_generation_rate: 1.0
    trajectory_publish_rate: 10.0
```

**Using Parameter Files:**
```bash
ros2 run uav_planning bioinspired_path_generator \
  --ros-args --params-file config/agricultural_field.yaml
```

## Real-World Configurations

### Crop Monitoring Configuration

For systematic crop health monitoring:

```yaml
crop_monitoring:
  ros__parameters:
    # Rectangular field
    x_min: 0.0
    x_max: 200.0
    y_min: 0.0
    y_max: 150.0
    z_min: 15.0
    z_max: 25.0
    
    # Moderate exploration for good coverage
    alpha: 1.5
    visit_threshold: 5.0
    velocity: 10.0
    
    # Slower generation for imaging
    waypoint_generation_rate: 0.2
    trajectory_publish_rate: 5.0
```

### Emergency Search Configuration

For rapid area coverage in emergency situations:

```yaml
emergency_search:
  ros__parameters:
    # Large search area
    x_min: -500.0
    x_max: 500.0
    y_min: -500.0
    y_max: 500.0
    z_min: 50.0
    z_max: 100.0
    
    # Aggressive long-range search
    alpha: 2.5
    visit_threshold: 10.0
    velocity: 20.0
    
    # Fast updates
    waypoint_generation_rate: 2.0
    trajectory_publish_rate: 20.0
```

### Precision Agriculture Configuration

For detailed field analysis:

```yaml
precision_agriculture:
  ros__parameters:
    # Typical field size
    x_min: -50.0
    x_max: 50.0
    y_min: -75.0
    y_max: 75.0
    z_min: 8.0
    z_max: 12.0
    
    # Conservative for detailed coverage
    alpha: 1.2
    visit_threshold: 2.0
    velocity: 6.0
    
    # Precise timing
    waypoint_generation_rate: 0.5
    trajectory_publish_rate: 15.0
```

## Dynamic Parameter Updates

Parameters can be updated at runtime using ROS 2 parameter services:

```bash
# Update flight boundaries
ros2 param set /bioinspired_path_generator x_max 150.0
ros2 param set /bioinspired_path_generator z_max 30.0

# Update algorithm behavior
ros2 param set /bioinspired_path_generator alpha 1.8
ros2 param set /bioinspired_path_generator velocity 12.0

# Get current parameters
ros2 param get /bioinspired_path_generator alpha
ros2 param list /bioinspired_path_generator
```

## Parameter Validation

The system validates parameters and will log warnings for invalid values:

### Boundary Validation
- `x_max` must be greater than `x_min`
- `y_max` must be greater than `y_min`
- `z_max` must be greater than `z_min`
- All altitude values must be positive

### Algorithm Validation
- `alpha` must be between 1.0 and 3.0
- `visit_threshold` must be positive
- `velocity` must be positive
- Update rates must be positive

## Advanced Configuration

### PX4-Specific Settings

When using PX4 integration, additional considerations:

```yaml
px4_integration:
  ros__parameters:
    # Coordinate frame considerations
    # PX4 uses NED (North-East-Down), system converts automatically
    
    # Conservative thresholds for real hardware
    visit_threshold: 3.0  # Larger threshold for GPS accuracy
    velocity: 8.0         # Conservative speed for safety
    
    # Timing for real hardware
    trajectory_publish_rate: 10.0  # Adequate for PX4
    waypoint_generation_rate: 0.5  # Allow time for waypoint execution
```

### Debugging Configuration

For development and debugging:

```yaml
debug_config:
  ros__parameters:
    # Small test area
    x_min: -5.0
    x_max: 5.0
    y_min: -5.0
    y_max: 5.0
    z_min: 2.0
    z_max: 3.0
    
    # Predictable behavior
    alpha: 1.5
    visit_threshold: 0.5
    velocity: 2.0
    
    # Fast updates for testing
    waypoint_generation_rate: 2.0
    trajectory_publish_rate: 20.0
```

## Environment Variables

Additional configuration through environment variables:

```bash
# Enable debug output
export ROS_LOG_LEVEL=DEBUG

# Set DDS domain (for multiple robots)
export ROS_DOMAIN_ID=42

# Custom QoS settings
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/qos_profiles.xml
```

## Troubleshooting Configuration

### Common Configuration Issues

1. **Waypoints outside boundaries**: Check min/max parameter consistency
2. **No waypoint generation**: Verify `enable_path_generation` is true
3. **Erratic flight patterns**: Adjust `alpha` parameter (try values closer to 1.5)
4. **UAV not reaching waypoints**: Increase `visit_threshold`

### Configuration Validation Script

Create a script to validate your configuration:

```python
#!/usr/bin/env python3
"""Validate UAV path planning configuration."""

def validate_config(params):
    """Validate configuration parameters."""
    errors = []
    
    # Boundary validation
    if params['x_max'] <= params['x_min']:
        errors.append("x_max must be greater than x_min")
    
    if params['y_max'] <= params['y_min']:
        errors.append("y_max must be greater than y_min")
    
    if params['z_max'] <= params['z_min']:
        errors.append("z_max must be greater than z_min")
    
    # Algorithm validation
    if not (1.0 < params['alpha'] <= 3.0):
        errors.append("alpha must be between 1.0 and 3.0")
    
    if params['velocity'] <= 0:
        errors.append("velocity must be positive")
    
    if params['visit_threshold'] <= 0:
        errors.append("visit_threshold must be positive")
    
    return errors

# Example usage
config = {
    'x_min': -10.0, 'x_max': 10.0,
    'y_min': -10.0, 'y_max': 10.0,
    'z_min': 2.0, 'z_max': 8.0,
    'alpha': 1.5,
    'velocity': 5.0,
    'visit_threshold': 1.2
}

errors = validate_config(config)
if errors:
    print("Configuration errors found:")
    for error in errors:
        print(f"  - {error}")
else:
    print("Configuration is valid!")
```

---

*Configuration guide last updated: January 5, 2025*
