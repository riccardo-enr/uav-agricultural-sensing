# UAV Agricultural Sensing

A ROS 2 package for autonomous UAV path planning in agricultural environments using butterfly-inspired Lévy flight patterns for optimal field coverage and sensing.

[![ROS 2](https://img.shields.io/badge/ROS-2%20Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Python](https://img.shields.io/badge/Python-3.12-green)](https://www.python.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04%20LTS-orange)](https://ubuntu.com/)
[![DevContainer](https://img.shields.io/badge/Dev-Container%20Ready-blue)](https://code.visualstudio.com/docs/remote/containers)
[![License](https://img.shields.io/badge/License-See%20LICENSE-lightgrey)](LICENSE)

## Overview

This project implements a bio-inspired path planning algorithm for agricultural UAVs based on butterfly foraging behavior. The system generates waypoints following Lévy flight patterns, which have been proven effective for exploration and coverage tasks in unknown or partially known environments.

## 📚 Documentation

Comprehensive documentation is available in the [`docs/`](docs/) folder:

- **[Installation Guide](docs/installation.md)** - Development container setup and manual installation
- **[Development Container Guide](docs/devcontainer.md)** - Working with development containers
- **[Quick Start Tutorial](docs/tutorials/quickstart.md)** - Get up and running in minutes
- **[Launch Files Guide](docs/launch_files.md)** - Using launch configurations for different scenarios
- **[Code Reference](docs/api/)** - Detailed code documentation
- **[Configuration Guide](docs/configuration.md)** - Parameter tuning and customization
- **[Examples](docs/examples/)** - Sample configurations and use cases
- **[Troubleshooting](docs/troubleshooting.md)** - Common issues and solutions

## Quick Start

This project uses **development containers** for a streamlined setup experience. All dependencies are pre-configured in the container environment.

### Development Container Setup (Recommended)

1. **Prerequisites**: 
   - [Visual Studio Code](https://code.visualstudio.com/)
   - [Docker](https://docs.docker.com/get-docker/)
   - [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

2. **Launch**:
   ```bash
   # Clone the repository
   git clone <repository-url> uav-agricultural-sensing
   cd uav-agricultural-sensing
   
   # Open in VS Code and reopen in container
   code .
   # Command Palette (Ctrl+Shift+P) -> "Dev Containers: Reopen in Container"
   ```

3. **Build and Run**:
   ```bash
   # Everything is pre-configured in the container!
   colcon build --packages-select uav_planning
   source install/setup.bash
   
   # Run the complete simulation
   ros2 launch uav_planning uav_simulation.launch.py
   
   # Or run just the path planner
   ros2 launch uav_planning path_planner.launch.py
   
   # Or try a demo configuration
   ros2 launch uav_planning demo.launch.py mode:=full_simulation
   ```

> 📖 **Pro Tip**: See the [Launch Files Guide](docs/launch_files.md) for detailed information about all available launch configurations and parameters.

### Manual Installation

For manual installation outside of the development container, see the [installation guide](docs/installation.md).

## Features

- **🦋 Butterfly-Inspired Lévy Flight**: Generates natural, efficient exploration patterns
- **🔗 ROS 2 Integration**: Full compatibility with ROS 2 ecosystem and PX4
- **🗺️ Configurable Boundaries**: Flexible field boundary constraints
- **⚡ Real-time Path Generation**: Dynamic waypoint generation with adjustable parameters
- **📊 Visualization Tools**: Comprehensive plotting and analysis capabilities
- **🌾 Agricultural Focus**: Optimized for crop monitoring and field surveying applications
- **🛩️ PX4 Compatibility**: Direct integration with PX4 autopilot via px4_msgs

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

- 📧 **Email**: giorgia.giacalone@polito.it, riccardo.enrico@polito.it
- 🐛 **Issues**: [GitHub Issues](https://github.com/your-username/uav-agricultural-sensing/issues)
- 📖 **Documentation**: [docs/](docs/)
- 💬 **Discussions**: [GitHub Discussions](https://github.com/your-username/uav-agricultural-sensing/discussions)

---

**Keywords**: UAV, Agricultural Sensing, Lévy Flight, Butterfly Algorithm, ROS 2, Path Planning, Autonomous Systems, Precision Agriculture, PX4