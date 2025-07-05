# Development Container Guide

This guide covers everything you need to know about working with the UAV Agricultural Sensing project in development containers.

## 🐳 What are Development Containers?

Development containers provide a fully configured development environment using Docker. This ensures that every contributor works with the exact same tools, dependencies, and configuration, eliminating "works on my machine" issues.

## 🚀 Quick Start

### Prerequisites

- [Visual Studio Code](https://code.visualstudio.com/)
- [Docker](https://docs.docker.com/get-docker/)
- [Dev Containers Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

### Launch Steps

1. **Clone and Open**:
   ```bash
   git clone <repository-url> uav-agricultural-sensing
   cd uav-agricultural-sensing
   code .
   ```

2. **Reopen in Container**:
   - Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on macOS)
   - Type: "Dev Containers: Reopen in Container"
   - Wait for the container to build (5-10 minutes first time)

3. **Start Developing**:
   ```bash
   # Build the packages
   colcon build --packages-select uav_interfaces uav_planning
   source install/setup.bash
   
   # Run the path generator
   ros2 run uav_planning bioinspired_path_generator --help
   ```

## 📁 Container Configurations

The project provides three development container configurations:

### 1. UAV Container (`uav`)
- **File**: `.devcontainer/uav/devcontainer.json`
- **Use Case**: Primary development environment
- **Features**: Full ROS 2 setup, PX4, basic GUI support
- **Resources**: ~4GB RAM, modest CPU

### 2. Desktop Container (`desktop`)
- **File**: `.devcontainer/desktop/devcontainer.json`
- **Use Case**: Heavy simulation work with full desktop
- **Features**: Complete desktop environment, GPU acceleration
- **Resources**: ~8GB RAM, GPU required

### 3. Desktop CPU Container (`desktop-cpu`)
- **File**: `.devcontainer/desktop-cpu/devcontainer.json`
- **Use Case**: Desktop environment without GPU
- **Features**: Full desktop, CPU-only rendering
- **Resources**: ~6GB RAM, no GPU required

## 🔧 Pre-installed Software

All containers include:

### System
- **Ubuntu 24.04 LTS** - Latest long-term support
- **Python 3.12** - Latest Python with modern features
- **Git** - Version control with user configuration
- **Build tools** - GCC, Make, CMake, etc.

### ROS 2 Environment
- **ROS 2 Jazzy Jalisco** - Latest LTS distribution
- **Colcon** - Build system
- **rosdep** - Dependency management
- **Common ROS 2 packages** - geometry_msgs, std_msgs, etc.

### UAV-Specific Tools
- **PX4 Autopilot** - Latest stable release
- **px4_msgs** - PX4 message definitions
- **Gazebo Garden** - Advanced simulation
- **microXRCE DDS Agent** - PX4 communication bridge

### Development Tools
- **VS Code Extensions**: ROS, Python, C++, XML, Docker
- **Python Packages**: NumPy, Matplotlib, SciPy
- **Debugging Tools**: GDB, Valgrind
- **Testing Framework**: pytest, gtest

## 💾 Persistent Storage

### Workspace Files
Your workspace files are automatically mounted and persist between container sessions.

### Container-specific Data
Some data is stored in Docker volumes:
- ROS 2 logs: `/opt/ros/jazzy/`
- Python packages: `/usr/local/lib/python3.12/`
- Build artifacts: `build/`, `install/`, `log/`

### Git Configuration
Configure Git inside the container:
```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

## 🎮 Common Commands

### Building
```bash
# Build all packages
colcon build

# Build specific packages
colcon build --packages-select uav_planning

# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Running
```bash
# Source workspace
source install/setup.bash

# Run path generator
ros2 run uav_planning bioinspired_path_generator

# Launch simulation
ros2 launch uav_planning uav_simulation.launch.py
```

### Testing
```bash
# Run all tests
colcon test

# Run specific tests
colcon test --packages-select uav_planning

# Show test results
colcon test-result --verbose
```

## 🖥️ GUI Applications

### X11 Forwarding (Linux)
```bash
# On host (outside container)
xhost +local:docker

# Test GUI in container
xclock  # Should show a clock window
gazebo  # Should open Gazebo
```

### Windows/macOS
GUI forwarding works automatically through VS Code's forwarding mechanism.

## 🔍 Troubleshooting

### Container Won't Start

**Issue**: "Failed to start container"
```bash
# Check Docker status
docker --version
docker ps

# Restart Docker
sudo systemctl restart docker  # Linux
# Or restart Docker Desktop
```

### Performance Issues

**Issue**: Container is slow
```bash
# Increase Docker memory limit
# Docker Desktop → Settings → Resources → Memory: 8GB+

# Check available resources
docker stats
htop
```

### Build Failures

**Issue**: Colcon build fails
```bash
# Clean build
rm -rf build/ install/ log/

# Update rosdep
rosdep update

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build with verbose output
colcon build --event-handlers console_direct+
```

### GUI Not Working

**Issue**: Gazebo/GUIs don't open
```bash
# Linux: Enable X11 forwarding
xhost +local:docker

# Check display variables
echo $DISPLAY
echo $WAYLAND_DISPLAY

# Test with simple GUI
xclock
```

## 🚀 Advanced Usage

### Custom Extensions
Add VS Code extensions in `.devcontainer/devcontainer.json`:
```json
"customizations": {
  "vscode": {
    "extensions": [
      "ms-python.python",
      "your-extension-id"
    ]
  }
}
```

### Environment Variables
Set custom environment variables:
```json
"containerEnv": {
  "ROS_DOMAIN_ID": "42",
  "CUSTOM_VAR": "value"
}
```

### Port Forwarding
Forward ports for web interfaces:
```json
"forwardPorts": [8080, 3000]
```

### Volume Mounts
Mount additional directories:
```json
"mounts": [
  "source=/host/path,target=/container/path,type=bind"
]
```

## 📈 Performance Tips

### Docker Optimization
```bash
# Increase shared memory (Linux)
# Add to /etc/sysctl.conf:
kernel.shmmax = 134217728

# Use faster filesystem (Windows/macOS)
# Use named volumes instead of bind mounts for build artifacts
```

### Container Optimization
```bash
# Parallel builds
colcon build --parallel-workers 4

# Compiler cache
export CCACHE_DIR=/tmp/ccache
colcon build --cmake-args -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
```

## 🆘 Getting Help

If you encounter issues with development containers:

1. Check the [Troubleshooting Guide](troubleshooting.md)
2. Verify Docker installation and resources
3. Try rebuilding the container: "Dev Containers: Rebuild Container"
4. Check [VS Code Dev Containers documentation](https://code.visualstudio.com/docs/remote/containers)
5. Create an issue with:
   - Container configuration used
   - Docker version and system specs
   - Full error messages
   - Steps to reproduce

---

**Benefits of Development Containers:**
- ✅ Zero setup time
- ✅ Consistent environment across all machines
- ✅ No dependency conflicts
- ✅ Easy to update and maintain
- ✅ Perfect for CI/CD integration

*Development Container Guide last updated: July 5, 2025*
