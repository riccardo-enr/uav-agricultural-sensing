# UAV Agricultural Sensing Documentation

Welcome to the comprehensive documentation for the UAV Agricultural Sensing project. This documentation provides detailed guides, tutorials, and reference materials to help you get started and make the most of the system.

## � Development Environment

This project uses **development containers** for a streamlined setup experience. All dependencies are pre-configured with:

- **Ubuntu 24.04 LTS** - Latest long-term support release
- **ROS 2 Jazzy Jalisco** - Latest ROS 2 LTS distribution
- **Python 3.12** - Modern Python with latest features
- **PX4 Autopilot** - Latest stable release
- **Gazebo Sunflower Field** - Simulation environment

## �📚 Documentation Structure

### Getting Started
- **[Installation Guide](installation.md)** - Development container setup and manual installation
- **[Quick Start Tutorial](tutorials/quickstart.md)** - Get up and running in 15 minutes
- **[Quick Start Guide](quick_start.md)** - Separated architecture quick start
- **[System Architecture](architecture.md)** - Understanding the system design
- **[Separated Architecture](separated_architecture.md)** - New modular architecture design

### User Guides
- **[Configuration Guide](configuration.md)** - Parameter tuning and customization
- **[Launch Files Guide](launch_files.md)** - Using the automated launch system
- **[Troubleshooting](troubleshooting.md)** - Common issues and solutions

### Developer Resources
- **[Code Reference](api/)** - Detailed code documentation
- **[Contributing Guide](contributing.md)** - How to contribute to the project
- **[Testing Guide](testing.md)** - Running and writing tests

### Examples & Tutorials
- **[Basic Usage Examples](examples/)** - Sample configurations and use cases
- **[Advanced Tutorials](tutorials/)** - In-depth guides for complex scenarios

### Reference
- **[Algorithm Details](algorithm_details.md)** - Deep dive into Lévy flight implementation
- **[ROS 2 Integration](ros2_integration.md)** - ROS 2 specific information
- **[PX4 Integration](px4_integration.md)** - PX4 autopilot integration details

## 🚀 Quick Navigation

| I want to...                       | Go to...                                        |
| ---------------------------------- | ----------------------------------------------- |
| Set up the development environment | [Installation Guide](installation.md)           |
| Run my first simulation            | [Quick Start Tutorial](tutorials/quickstart.md) |
| Customize parameters               | [Configuration Guide](configuration.md)         |
| Understand the algorithm           | [Algorithm Details](algorithm_details.md)       |
| Fix a problem                      | [Troubleshooting](troubleshooting.md)           |
| Contribute code                    | [Contributing Guide](contributing.md)           |
| See examples                       | [Examples](examples/)                           |

## 🎯 Key Concepts

Before diving into the documentation, familiarize yourself with these key concepts:

- **Development Containers**: Pre-configured, isolated development environments
- **Lévy Flight**: A type of random walk where step lengths follow a heavy-tailed probability distribution
- **Butterfly Explorer**: Our bio-inspired algorithm based on butterfly foraging behavior
- **Waypoint Generation**: The process of creating navigation targets for the UAV
- **PX4 Integration**: Direct communication with PX4 autopilot via px4_msgs
## 📈 Documentation Status

| Document           | Status        | Last Updated | Notes                      |
| ------------------ | ------------- | ------------ | -------------------------- |
| Installation Guide | ✅ Complete    | 2025-07-05   | Updated for dev containers |
| Quick Start        | ✅ Complete    | 2025-07-05   | Container-based workflow   |
| Code Reference     | ✅ Complete    | 2025-07-05   | ROS 2 Jazzy compatible     |
| Configuration      | ✅ Complete    | 2025-07-05   | Python 3.12 ready          |
| Troubleshooting    | 🔄 In Progress | 2025-07-05   | Container-specific fixes   |

## � Why Development Containers?

This project leverages development containers to provide:

- ✅ **Zero Setup Time**: No manual dependency installation
- ✅ **Consistent Environment**: Same setup across all machines
- ✅ **Isolation**: No conflicts with host system
- ✅ **Reproducibility**: Identical environment for all contributors
- ✅ **Latest Tools**: Ubuntu 24.04, ROS 2 Jazzy, Python 3.12

## �💡 Need Help?

If you can't find what you're looking for in the documentation:

1. Check the [Troubleshooting Guide](troubleshooting.md)
2. Search existing [GitHub Issues](https://github.com/your-username/uav-agricultural-sensing/issues)
3. Create a new issue with the `documentation` label
4. Contact the maintainer at riccardo.enrico97@gmail.com

## 🤝 Contributing to Documentation

We welcome improvements to the documentation! See the [Contributing Guide](contributing.md) for details on how to:

- Report documentation issues
- Suggest improvements
- Submit documentation updates
- Translate documentation

---

*This documentation is generated for UAV Agricultural Sensing v1.0 - Ubuntu 24.04, ROS 2 Jazzy, Python 3.12*
