# Installation Guide

This guide provides setup instructions for the UAV Agricultural Sensing system. The project is designed to work seamlessly with **development containers**, providing a pre-configured environment with all dependencies.

## 🐳 Development Container Setup (Recommended)

The easiest way to get started is using the pre-configured development containers. All dependencies including ROS 2 Jazzy, Python 3.12, PX4, and Gazebo are already installed and configured.

### Prerequisites

- **[Visual Studio Code](https://code.visualstudio.com/)** - Primary development environment
- **[Docker](https://docs.docker.com/get-docker/)** - Container runtime
- **[Dev Containers Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)** - VS Code extension

### Quick Start

1. **Clone the repository**:
   ```bash
   git clone <repository-url> uav-agricultural-sensing
   cd uav-agricultural-sensing
   ```

2. **Open in VS Code**:
   ```bash
   code .
   ```

3. **Reopen in Container**:
   - Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on macOS)
   - Type: "Dev Containers: Reopen in Container"
   - Select the command and wait for the container to build

4. **Build the package**:
   ```bash
   # Inside the container terminal
   colcon build --packages-select uav_planning uav_interfaces
   source install/setup.bash
   ```

5. **Verify installation**:
   ```bash
   # Test the path generator
   ros2 run uav_planning bioinspired_path_generator --help
   ```

### Container Features

✅ **Pre-installed Software**:
- Ubuntu 24.04 LTS
- ROS 2 Jazzy Jalisco
- Python 3.12
- PX4 Autopilot (latest)
- Gazebo Garden
- All project dependencies

✅ **Pre-configured Environment**:
- ROS 2 workspace setup
- Environment variables
- Python virtual environment
- Git configuration
- VS Code extensions

## 🚀 Next Steps

After successful installation:

1. 📖 Follow the [Quick Start Tutorial](tutorials/quickstart.md)
2. ⚙️ Review the [Configuration Guide](configuration.md)
3. 🎯 Explore the [Examples](examples/)
4. 🔍 Read the [Architecture Documentation](architecture.md)

## 🆘 Getting Help

If you encounter issues:

1. 📋 Check the [Troubleshooting Guide](troubleshooting.md)
2. 🔍 Search [GitHub Issues](https://github.com/your-username/uav-agricultural-sensing/issues)
3. 🐛 Create a new issue with:
   - Operating system and version
   - Container configuration (if applicable)
   - Full error messages
   - Steps to reproduce

---

**Why Development Containers?**

- ✅ **Consistent Environment**: Same setup across all machines
- ✅ **Zero Installation**: No manual dependency management
- ✅ **Isolation**: No conflicts with host system
- ✅ **Reproducibility**: Identical development environment for all contributors
- ✅ **Easy Updates**: Container updates include all dependencies

