# Installation Guide

This guide provides step-by-step instructions for installing the UAV Agricultural Sensing system on Ubuntu 24.04 LTS.

## Prerequisites

### System Requirements

- **Operating System**: Ubuntu 20.04 LTS or Ubuntu 22.04 LTS (Ubuntu 24.04 LTS recommended)
- **Architecture**: x86_64 (ARM64 support experimental)
- **RAM**: Minimum 8GB (16GB recommended for simulation)
- **Storage**: At least 20GB free space
- **GPU**: Optional but recommended for Gazebo simulation

### Required Software Versions

| Software | Minimum Version | Recommended      |
| -------- | --------------- | ---------------- |
| ROS 2    | Humble          | Humble Hawksbill |
| Python   | 3.8             | 3.10+            |
| Gazebo   | 11.0            | Latest           |
| PX4      | 1.13            | 1.14+            |

## Installation Steps

### 1. ROS 2 Installation

If you don't have ROS 2 installed, follow the official installation guide:

```bash
# Add ROS 2 apt repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop python3-argcomplete
```

### 2. Install Dependencies

```bash
# Install Python dependencies
sudo apt install python3-pip python3-pytest-cov ros-dev-tools

# Install required Python packages
pip3 install numpy matplotlib scipy

# Install ROS 2 packages
sudo apt install ros-humble-tf2-tools ros-humble-tf2-ros ros-humble-geometry-msgs
```

### 3. Install PX4 and Dependencies

#### Option A: Quick Installation (Recommended)

```bash
# Clone PX4 repository
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# Run the setup script (this will take 10-15 minutes)
bash ./Tools/setup/ubuntu.sh

# Build PX4 for simulation
make px4_sitl_default gazebo-classic
```

#### Option B: Manual Installation

```bash
# Install PX4 dependencies
sudo apt install python3-dev python3-pip python3-numpy python3-yaml
pip3 install --user pyserial empy toml numpy packaging jinja2

# Install Java (required for some PX4 tools)
sudo apt install openjdk-11-jdk

# Install Gazebo Classic
sudo apt install gazebo libgazebo-dev
```

### 4. Install px4_msgs

```bash
# Create workspace if it doesn't exist
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone px4_msgs
git clone https://github.com/PX4/px4_msgs.git

# Build px4_msgs
cd ~/ros2_ws
colcon build --packages-select px4_msgs
source install/setup.bash
```

### 5. Install microXRCE DDS Agent

```bash
# Install from apt (easiest method)
sudo apt install ros-humble-micro-xrce-dds-agent

# OR build from source for latest features
cd ~/ros2_ws/src
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd ~/ros2_ws
colcon build --packages-select micro_xrce_dds_agent
```

### 6. Clone and Build UAV Agricultural Sensing

```bash
# Navigate to ROS 2 workspace
cd ~/ros2_ws/src

# Clone the repository
git clone <repository-url> uav-agricultural-sensing

# Install dependencies using rosdep
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select uav_planning

# Source the workspace
source install/setup.bash
```

## Post-Installation Setup

### 1. Environment Setup

Add these lines to your `~/.bashrc` file:

```bash
# ROS 2 setup
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# PX4 setup
export PX4_DIR=~/PX4-Autopilot
source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default

# Gazebo setup
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
```

Reload your environment:
```bash
source ~/.bashrc
```

### 2. Verify Installation

Test each component individually:

```bash
# Test ROS 2
ros2 --version

# Test PX4 SITL
cd ~/PX4-Autopilot
make px4_sitl none_iris

# Test microXRCE DDS Agent (in another terminal)
micro-xrce-dds-agent udp4 -p 8888

# Test UAV Planning package
ros2 run uav_planning bioinspired_path_generator --help
```

## Optional Components

### Gazebo Garden (Alternative to Classic)

```bash
# Install Gazebo Garden
sudo apt-get update
sudo apt-get install lsb-release wget gnupg

sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
sudo apt-get install gz-garden
```

### MAVROS (Alternative to direct PX4 integration)

```bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# Install GeographicLib datasets
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

## Troubleshooting Installation Issues

### Common Issues

#### 1. Permission Denied Errors
```bash
# Fix permissions for serial devices
sudo usermod -a -G dialout $USER
# Log out and log back in
```

#### 2. Gazebo Models Not Found
```bash
# Download Gazebo models
cd ~/.gazebo
wget -r -np -nH --cut-dirs=2 http://models.gazebosim.org/
```

#### 3. PX4 Build Errors
```bash
# Clean and rebuild PX4
cd ~/PX4-Autopilot
make clean
make distclean
make px4_sitl_default gazebo-classic
```

#### 4. Python Package Issues
```bash
# Reinstall Python packages in virtual environment
python3 -m venv ~/venv
source ~/venv/bin/activate
pip install numpy matplotlib scipy
```

### Verification Commands

Test your installation with these commands:

```bash
# 1. Check ROS 2 environment
echo $ROS_DISTRO
ros2 pkg list | grep uav_planning

# 2. Check PX4 installation
ls ~/PX4-Autopilot/build/px4_sitl_default/bin/px4

# 3. Check px4_msgs
ros2 interface list | grep px4_msgs

# 4. Test complete simulation (should open Gazebo and show UAV)
ros2 launch uav_planning uav_simulation.launch.py headless:=false
```

## Performance Optimization

### For Better Simulation Performance

```bash
# Install proprietary NVIDIA drivers (if applicable)
sudo ubuntu-drivers autoinstall

# Increase shared memory for DDS
echo 'kernel.shmmax = 134217728' | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

### For Real Hardware Deployment

```bash
# Install additional real-time kernel (optional)
sudo apt install linux-lowlatency

# Configure CPU governor for performance
sudo apt install cpufrequtils
echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils
sudo systemctl restart cpufrequtils
```

## Next Steps

After successful installation:

1. Follow the [Quick Start Tutorial](tutorials/quickstart.md)
2. Review the [Configuration Guide](configuration.md)
3. Explore the [Examples](examples/)

## Getting Help

If you encounter issues during installation:

1. Check the [Troubleshooting Guide](troubleshooting.md)
2. Search existing [GitHub Issues](https://github.com/your-username/uav-agricultural-sensing/issues)
3. Create a new issue with:
   - Your operating system version
   - Error messages (full output)
   - Steps you've already tried

---

*Installation guide last updated: January 5, 2025*
