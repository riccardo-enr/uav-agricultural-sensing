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

### Available Container Configurations

The project provides three development container options:

| Container     | Description                               | Use Case                     |
| ------------- | ----------------------------------------- | ---------------------------- |
| `uav`         | Full UAV development with GUI support     | Main development, simulation |
| `desktop`     | Full desktop environment with GPU support | Heavy simulation workloads   |
| `desktop-cpu` | CPU-only desktop environment              | Development without GPU      |

To switch containers, modify `.devcontainer/devcontainer.json` to reference the desired configuration.

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

## 🖥️ Manual Installation (Advanced Users)

If you prefer not to use development containers, you can install the system manually on Ubuntu 24.04 LTS.

### System Requirements

- **Operating System**: Ubuntu 24.04 LTS
- **Architecture**: x86_64 or ARM64
- **RAM**: Minimum 8GB (16GB recommended for simulation)
- **Storage**: At least 20GB free space
- **GPU**: Optional but recommended for Gazebo simulation

### Required Software Versions

| Software | Version       | Notes                          |
| -------- | ------------- | ------------------------------ |
| Ubuntu   | 24.04 LTS     | Primary supported platform     |
| ROS 2    | Jazzy Jalisco | Latest LTS release             |
| Python   | 3.12          | System default on Ubuntu 24.04 |
| Gazebo   | Garden        | Latest stable release          |
| PX4      | 1.14+         | Latest stable branch           |

### Manual Installation Steps

#### 1. Install ROS 2 Jazzy

```bash
# Ensure UTF-8 locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2 Jazzy
sudo apt update
sudo apt install ros-jazzy-desktop python3-argcomplete ros-dev-tools
```

#### 2. Install Python Dependencies

```bash
# Install pip and development tools
sudo apt install python3-pip python3-pytest-cov python3-venv

# Create and activate virtual environment
python3 -m venv ~/venv/uav-sensing
source ~/venv/uav-sensing/bin/activate

# Install Python packages
pip install numpy matplotlib scipy
```

#### 3. Install PX4 Autopilot

```bash
# Clone PX4 repository
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# Run setup script
bash ./Tools/setup/ubuntu.sh

# Build for simulation
make px4_sitl_default gazebo-classic
```

#### 4. Install Additional Dependencies

```bash
# Install ROS 2 packages
sudo apt install \
    ros-jazzy-tf2-tools \
    ros-jazzy-tf2-ros \
    ros-jazzy-geometry-msgs \
    ros-jazzy-micro-xrce-dds-agent

# Install px4_msgs
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd ~/ros2_ws
colcon build --packages-select px4_msgs
```

#### 5. Build UAV Agricultural Sensing

```bash
# Clone the repository
cd ~/ros2_ws/src
git clone <repository-url> uav-agricultural-sensing

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the packages
colcon build --packages-select uav_interfaces uav_planning
source install/setup.bash
```

#### 6. Environment Setup

Add to your `~/.bashrc`:

```bash
# Python virtual environment
source ~/venv/uav-sensing/bin/activate

# ROS 2 setup
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# PX4 setup
export PX4_DIR=~/PX4-Autopilot
source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash \
    ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
```

## 🧪 Verification

Test your installation:

```bash
# Test ROS 2
ros2 --version  # Should show "ros2 cli version: jazzy"

# Test Python environment
python3 --version  # Should show Python 3.12.x

# Test UAV Planning package
ros2 run uav_planning bioinspired_path_generator --help

# Test complete simulation (if using GUI)
ros2 launch uav_planning uav_simulation.launch.py headless:=false
```

## 🔧 Troubleshooting

### Development Container Issues

**Container fails to build**:
```bash
# Clear Docker cache and rebuild
docker system prune -a
# Reopen in container again
```

**Permission issues**:
```bash
# Fix file permissions (run on host)
sudo chown -R $USER:$USER .
```

**X11 forwarding not working**:
```bash
# On host system (Linux)
xhost +local:docker
```

### Manual Installation Issues

**ROS 2 not found**:
```bash
# Verify installation
dpkg -l | grep ros-jazzy
source /opt/ros/jazzy/setup.bash
```

**Python package conflicts**:
```bash
# Use virtual environment
python3 -m venv ~/venv/uav-sensing
source ~/venv/uav-sensing/bin/activate
pip install --upgrade pip
```

**PX4 build errors**:
```bash
# Clean and rebuild
cd ~/PX4-Autopilot
make clean && make distclean
bash ./Tools/setup/ubuntu.sh
make px4_sitl_default gazebo-classic
```

## 📈 Performance Optimization

### For Development Containers

```bash
# Increase Docker memory limit (in Docker Desktop)
# Settings → Resources → Memory: 8GB+

# Use faster filesystem (Windows/macOS)
# Use named volumes for better performance
```

### For Manual Installation

```bash
# Install NVIDIA drivers for GPU acceleration
sudo ubuntu-drivers autoinstall

# Optimize CPU performance
sudo apt install cpufrequtils
echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils
```

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

*Installation guide last updated: July 5, 2025*

