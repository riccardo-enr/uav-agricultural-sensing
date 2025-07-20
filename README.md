# UAV Agricultural Sensing

ROS 2 package for autonomous UAV path planning in agricultural environments using butterfly-inspired Lévy flight patterns.

## Prerequisites

- ROS 2 Jazzy
- PX4 Autopilot
- Gazebo Simulation

For convenience, Dockerfiles and development containers are provided to simplify setup. The desktop devcontainer is the primary development environment - the other containers are available but not extensively tested or maintained.

## Quick Start

```bash
# Clone and open in VS Code
git clone <repository-url> uav-agricultural-sensing
cd uav-agricultural-sensing
code .

# Reopen in container (Ctrl+Shift+P -> "Dev Containers: Reopen in Container")

# Build and run
colcon build --packages-select uav_planning
source install/setup.bash
ros2 launch uav_planning uav_simulation.launch.py
```
