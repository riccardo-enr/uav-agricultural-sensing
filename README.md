# UAV Agricultural Sensing

A ROS 2 package for autonomous UAV path planning in agricultural environments using butterfly-inspired Lévy flight patterns for optimal field coverage and sensing.

[![ROS 2](https://img.shields.io/badge/ROS-2%20Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.8%2B-green)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-See%20LICENSE-lightgrey)](LICENSE)

## Overview

This project implements a bio-inspired path planning algorithm for agricultural UAVs based on butterfly foraging behavior. The system generates waypoints following Lévy flight patterns, which have been proven effective for exploration and coverage tasks in unknown or partially known environments.

## 📚 Documentation

Comprehensive documentation is available in the [`docs/`](docs/) folder:

- **[Installation Guide](docs/installation.md)** - Step-by-step setup instructions
- **[Quick Start Tutorial](docs/tutorials/quickstart.md)** - Get up and running in minutes
- **[Code Reference](docs/api/)** - Detailed code documentation
- **[Configuration Guide](docs/configuration.md)** - Parameter tuning and customization
- **[Examples](docs/examples/)** - Sample configurations and use cases
- **[Troubleshooting](docs/troubleshooting.md)** - Common issues and solutions

## Quick Start

1. **Installation**: Follow the [installation guide](docs/installation.md)
2. **Basic Usage**: See the [quick start tutorial](docs/tutorials/quickstart.md)
3. **Configuration**: Customize parameters using the [configuration guide](docs/configuration.md)

```bash
# Build the package
colcon build --packages-select uav_planning
source install/setup.bash

# Run the complete simulation
ros2 launch uav_planning uav_simulation.launch.py

# Or run just the path planner
ros2 run uav_planning bioinspired_path_generator
```

## Features

## Features

- **🦋 Butterfly-Inspired Lévy Flight**: Generates natural, efficient exploration patterns
- **🔗 ROS 2 Integration**: Full compatibility with ROS 2 ecosystem and PX4/MAVROS
- **🗺️ Configurable Boundaries**: Flexible field boundary constraints
- **⚡ Real-time Path Generation**: Dynamic waypoint generation with adjustable parameters
- **📊 Visualization Tools**: Comprehensive plotting and analysis capabilities
- **🌾 Agricultural Focus**: Optimized for crop monitoring and field surveying applications
- **🛩️ PX4 Compatibility**: Direct integration with PX4 autopilot via px4_msgs

## System Architecture

## System Architecture

The system consists of several key components:

1. **🧠 Core Algorithm** (`butterfly.py`): Pure Python implementation of Lévy flight patterns
2. **🎯 Path Generator** (`bioinspired_path_generator.py`): ROS 2 node with PX4 integration
3. **📡 Communication Layer**: Direct PX4 communication via px4_msgs
4. **🎮 Launch System**: Automated simulation and real-flight configurations

For detailed architecture information, see [docs/architecture.md](docs/architecture.md).

## Repository Structure

```
uav-agricultural-sensing/
├── docs/                     # 📚 Documentation
│   ├── api/                 # API reference
│   ├── tutorials/           # Step-by-step guides
│   └── examples/            # Usage examples
├── src/uav_planning/        # 🎯 Main package
│   ├── launch/             # Launch configurations
│   ├── uav_planning/       # Python modules
│   └── test/               # Unit tests
├── Bibliography/            # 📖 Research papers
└── log/                    # 📊 Runtime logs
```

For comprehensive setup instructions, parameter tuning, and advanced usage, please refer to the documentation in the [`docs/`](docs/) folder.

## Contributing

We welcome contributions! Please see [docs/contributing.md](docs/contributing.md) for guidelines.

## License

This project is licensed under the terms specified in the [LICENSE](LICENSE) file.

## Citation

If you use this work in your research, please cite:

```bibtex
@software{uav_agricultural_sensing,
  title={UAV Agricultural Sensing: Bio-inspired Path Planning},
  author={Your Name},
  year={2025},
  url={https://github.com/your-username/uav-agricultural-sensing}
}
```

## Support & Contact

- 📧 **Email**: riccardo.enrico97@gmail.com
- 🐛 **Issues**: [GitHub Issues](https://github.com/your-username/uav-agricultural-sensing/issues)
- 📖 **Documentation**: [docs/](docs/)
- 💬 **Discussions**: [GitHub Discussions](https://github.com/your-username/uav-agricultural-sensing/discussions)

---

**Keywords**: UAV, Agricultural Sensing, Lévy Flight, Butterfly Algorithm, ROS 2, Path Planning, Autonomous Systems, Precision Agriculture, PX4