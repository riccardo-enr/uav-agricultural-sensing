# UAV Agricultural Sensing

A ROS 2 package for autonomous UAV path planning in agricultural environments using butterfly-inspired Lévy flight patterns for optimal field coverage and sensing.

## Overview

This project implements a bio-inspired path planning algorithm for agricultural UAVs based on butterfly foraging behavior. The system generates waypoints following Lévy flight patterns, which have been proven effective for exploration and coverage tasks in unknown or partially known environments.

### Key Features

- **Butterfly-Inspired Lévy Flight**: Generates natural, efficient exploration patterns
- **ROS 2 Integration**: Full compatibility with ROS 2 ecosystem and MAVROS
- **Configurable Boundaries**: Flexible field boundary constraints
- **Real-time Path Generation**: Dynamic waypoint generation with adjustable parameters
- **Visualization Tools**: Comprehensive plotting and analysis capabilities
- **Agricultural Focus**: Optimized for crop monitoring and field surveying applications

## Architecture

The system consists of two main components:

1. **ButterflyGenerator** (`butterfly_generator.py`): Core algorithm implementation
   - Pure Python class, ROS-independent
   - Implements Lévy flight distribution with Pareto variate
   - Handles boundary constraints and state management

2. **ButterflyPathNode** (`uav_planning_node.py`): ROS 2 wrapper
   - ROS 2 node that publishes waypoints to `/mavros/setpoint_position/local`
   - Parameter server integration for runtime configuration
   - Timer-based waypoint generation at 1 Hz

## Installation

### Prerequisites

- ROS 2 (Humble or later)
- Python 3.8+
- MAVROS (for UAV communication)
- matplotlib (for visualization)
- numpy (for numerical operations)

### Build Instructions

1. Clone the repository into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository-url> uav-agricultural-sensing
```

2. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
colcon build --packages-select uav_planning
source install/setup.bash
```

## Usage

### Running the ROS 2 Node

Launch the butterfly path planner node:

```bash
ros2 run uav_planning uav_planning_node
```

### Configuration Parameters

The node accepts the following ROS 2 parameters:

| Parameter    | Type   | Default | Description                             |
| ------------ | ------ | ------- | --------------------------------------- |
| `x_min`      | double | 0.0     | Minimum X boundary (meters)             |
| `x_max`      | double | 10.0    | Maximum X boundary (meters)             |
| `y_min`      | double | 0.0     | Minimum Y boundary (meters)             |
| `y_max`      | double | 5.0     | Maximum Y boundary (meters)             |
| `z_min`      | double | 1.0     | Minimum altitude (meters)               |
| `z_max`      | double | 2.0     | Maximum altitude (meters)               |
| `alpha`      | double | 1.5     | Lévy flight shape parameter (1 < α ≤ 3) |
| `step_scale` | double | 1.0     | Step size scaling factor                |

### Example Launch with Custom Parameters

```bash
ros2 run uav_planning uav_planning_node \
  --ros-args \
  -p x_min:=0.0 \
  -p x_max:=50.0 \
  -p y_min:=0.0 \
  -p y_max:=30.0 \
  -p z_min:=2.0 \
  -p z_max:=10.0 \
  -p alpha:=1.8 \
  -p step_scale:=2.5
```

### Standalone Testing

For algorithm validation without ROS 2:

```bash
cd src/uav_planning/test
python3 test_butterfly_generator.py
```

This will generate comprehensive visualizations including:
- 2D top-down path view
- 3D trajectory visualization
- Altitude profile analysis
- Step size distribution histograms
- Boundary constraint validation

## Algorithm Details

### Lévy Flight Implementation

The butterfly generator uses a Pareto distribution to create Lévy flights:

```python
step = step_scale * random.paretovariate(alpha)
theta = random.uniform(0, 2 * math.pi)
```

- **α (alpha)**: Shape parameter controlling flight behavior
  - α ≈ 1.2: More conservative, localized search
  - α ≈ 1.5: Balanced exploration/exploitation
  - α ≈ 2.0: More aggressive, long-range movements

- **step_scale**: Global scaling factor for all movements

### Boundary Handling

The system implements hard boundary constraints using a clipping function:

```python
def clip(self, val, vmin, vmax):
    return max(min(val, vmax), vmin)
```

All waypoints are guaranteed to remain within the specified operational area.

## Applications

### Agricultural Use Cases

- **Crop Health Monitoring**: Systematic coverage for disease detection
- **Irrigation Management**: Water stress assessment across fields
- **Yield Estimation**: Pre-harvest crop counting and analysis
- **Pest Detection**: Early identification of infestations
- **Soil Analysis**: Mapping soil properties and conditions

### Advantages of Lévy Flight Patterns

- **Efficient Coverage**: Optimal balance between local and global search
- **Natural Movement**: Mimics biological foraging strategies
- **Scalable**: Adapts to different field sizes and shapes
- **Robust**: Continues operation even with partial failures

## Testing and Validation

The package includes comprehensive tests:

### Test Suite (`test_butterfly_generator.py`)

1. **Basic Generation Test**: Validates waypoint generation and plotting
2. **Multiple Configuration Test**: Compares different parameter settings
3. **Boundary Behavior Test**: Verifies constraint enforcement

Run tests:
```bash
cd src/uav_planning/test
python3 test_butterfly_generator.py
```

### Expected Outputs

- Path visualization plots (2D and 3D)
- Statistical analysis of generated paths
- Boundary constraint validation reports
- Step size distribution analysis

## Integration with MAVROS

The node publishes to `/mavros/setpoint_position/local` topic, compatible with:

- PX4 Flight Stack
- ArduPilot
- Other MAVROS-compatible autopilots

### Message Format

```
geometry_msgs/PoseStamped:
  header:
    stamp: current_time
    frame_id: "map"
  pose:
    position: {x, y, z}
    orientation: {0, 0, 0, 1}  # Neutral orientation
```

## Contributing

Contributions are welcome! Please follow these guidelines:

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Ensure all tests pass
5. Submit a pull request

### Code Style

- Follow PEP 8 for Python code
- Use docstrings for all classes and functions
- Remove superfluous comments (per ROS 2 coding standards)
- Prefix debug topics with `/debug/`
- Use `~/` prefix for node-specific topics

## License

This project is licensed under the [LICENSE](LICENSE) file in the root directory.

## Support

For questions, issues, or feature requests, please:

1. Check the existing issues on GitHub
2. Create a new issue with detailed information
3. Contact the maintainer at riccardo.enrico97@gmail.com

---

**Keywords**: UAV, Agricultural Sensing, Lévy Flight, Butterfly Algorithm, ROS 2, Path Planning, Autonomous Systems, Precision Agriculture