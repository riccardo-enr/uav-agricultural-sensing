# Example Configurations

This directory contains example configurations for different agricultural and surveillance scenarios using the UAV Agricultural Sensing system.

## Available Examples

### 🌾 Agricultural Scenarios

- **[Small Farm Monitoring](small_farm.yaml)** - 20-acre field coverage
- **[Large Commercial Farm](large_farm.yaml)** - 500-acre industrial agriculture
- **[Greenhouse Inspection](greenhouse.yaml)** - Indoor/covered crop monitoring
- **[Orchard Survey](orchard.yaml)** - Tree crop inspection with altitude variation
- **[Vineyard Monitoring](vineyard.yaml)** - Row crop inspection patterns

### 🏭 Industrial Applications

- **[Solar Farm Inspection](solar_farm.yaml)** - Photovoltaic panel monitoring
- **[Infrastructure Survey](infrastructure.yaml)** - Large area infrastructure inspection
- **[Emergency Search](emergency_search.yaml)** - Rapid area coverage for search and rescue

### 🧪 Research & Development

- **[Algorithm Testing](algorithm_test.yaml)** - Controlled environment for algorithm validation
- **[Performance Benchmarking](benchmark.yaml)** - Standardized performance testing
- **[Debug Configuration](debug.yaml)** - Development and troubleshooting setup

## Quick Start

To use any example configuration:

```bash
# Option 1: Direct launch with parameters
ros2 launch uav_planning uav_simulation.launch.py \
    --ros-args --params-file docs/examples/small_farm.yaml

# Option 2: Run node with configuration file
ros2 run uav_planning bioinspired_path_generator \
    --ros-args --params-file docs/examples/greenhouse.yaml

# Option 3: Load into running system
ros2 param load /bioinspired_path_generator docs/examples/orchard.yaml
```

## Configuration Categories

### Conservative Configurations
- **Alpha ≤ 1.5**: More predictable, systematic coverage
- **Lower velocities**: Precise, detailed inspection
- **Smaller areas**: Intensive monitoring applications

### Aggressive Configurations  
- **Alpha ≥ 1.8**: Rapid area coverage, efficient exploration
- **Higher velocities**: Quick surveys, time-critical applications
- **Larger areas**: Extensive monitoring applications

### Specialized Configurations
- **Custom boundaries**: Non-rectangular areas, obstacle avoidance
- **Multi-altitude**: Terrain following, 3D inspection
- **High-precision**: Sub-meter accuracy requirements

## Usage Examples

### Switching Between Configurations

```bash
# Start with small farm configuration
ros2 launch uav_planning uav_simulation.launch.py \
    --ros-args --params-file docs/examples/small_farm.yaml

# Switch to greenhouse configuration at runtime
ros2 param load /bioinspired_path_generator docs/examples/greenhouse.yaml

# Reset to default parameters
ros2 param load /bioinspired_path_generator src/uav_planning/config/default.yaml
```

### Creating Custom Configurations

Based on existing examples:

```bash
# Copy and modify an existing configuration
cp docs/examples/small_farm.yaml my_custom_config.yaml

# Edit parameters as needed
nano my_custom_config.yaml

# Test your configuration
ros2 launch uav_planning uav_simulation.launch.py \
    --ros-args --params-file my_custom_config.yaml
```

## Configuration Validation

Each example includes validation criteria:

- **Boundary Consistency**: min < max for all dimensions
- **Safety Limits**: Reasonable altitude and velocity constraints
- **Performance Optimization**: Balanced parameters for intended use case
- **Hardware Compatibility**: Suitable for target UAV platform

## Contributing Examples

To contribute new example configurations:

1. Create your configuration file following the naming convention
2. Test thoroughly in simulation and (if possible) real hardware
3. Document the intended use case and expected performance
4. Submit a pull request with your example and documentation

---

*Examples directory last updated: January 5, 2025*
