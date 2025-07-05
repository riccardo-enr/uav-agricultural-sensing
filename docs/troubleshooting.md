# Troubleshooting Guide

This guide helps you diagnose and fix common issues with the UAV Agricultural Sensing system in development containers and manual installations.

## � Development Container Issues

### Container Won't Start

**Problem**: VS Code fails to start the development container

**Solution**:
```bash
# Check Docker status
docker --version
docker ps

# Clear Docker cache and rebuild
docker system prune -a
docker volume prune

# Try reopening in container again
```

### Container Performance Issues

**Problem**: Container runs slowly or freezes

**Solution**:
```bash
# Increase Docker memory (Docker Desktop)
# Settings → Resources → Memory: 8GB+

# On Linux, check system resources
htop
df -h

# Restart Docker service
sudo systemctl restart docker
```

### X11/GUI Issues

**Problem**: Gazebo or GUI applications don't display

**Solution**:
```bash
# On Linux host
xhost +local:docker

# Check display environment in container
echo $DISPLAY
echo $WAYLAND_DISPLAY

# Test GUI forwarding
xclock  # Should show a clock window
```

## �🚨 Quick Diagnostic Checklist

When things aren't working, run through this checklist first:

### Environment Check
```bash
# 1. Check ROS 2 environment
echo $ROS_DISTRO  # Should be 'jazzy'
echo $ROS_DOMAIN_ID  # Should be a number (or empty)

# 2. Check Python version
python3 --version  # Should be 3.12.x

# 3. Check package installation
ros2 pkg list | grep uav_planning

# 4. Check workspace sourcing
echo $AMENT_PREFIX_PATH | grep ros2_ws

# 5. Check PX4 environment (if manual install)
echo $PX4_DIR  # Should point to PX4-Autopilot directory
ls $PX4_DIR/build/px4_sitl_default/bin/px4
```

### System Health Check
```bash
# 6. Check running nodes
ros2 node list

# 6. Check active topics
ros2 topic list | grep -E "(fmu|bioinspired)"

# 7. Check parameters
ros2 param list /bioinspired_path_generator 2>/dev/null || echo "Node not running"
```

## 📋 Common Issues and Solutions

### Issue 1: Package Not Found

**Symptoms:**
```
Package 'uav_planning' not found
```

**Solutions:**

1. **Rebuild the package:**
```bash
cd ~/ros2_ws
colcon build --packages-select uav_planning
source install/setup.bash
```

2. **Check package location:**
```bash
find ~/ros2_ws -name "package.xml" | grep uav_planning
```

3. **Verify workspace structure:**
```bash
ls ~/ros2_ws/src/uav-agricultural-sensing/src/uav_planning/
```

### Issue 2: UAV Not Moving in Simulation

**Symptoms:**
- Gazebo shows UAV but it doesn't move
- No waypoint generation messages

**Diagnosis:**
```bash
# Check if path generation is enabled
ros2 param get /bioinspired_path_generator enable_path_generation

# Check if node is publishing waypoints
ros2 topic hz /bioinspired_path_generator/current_waypoint

# Check vehicle odometry
ros2 topic echo /fmu/out/vehicle_odometry --field position
```

**Solutions:**

1. **Enable path generation:**
```bash
ros2 param set /bioinspired_path_generator enable_path_generation true
```

2. **Check vehicle arming:**
```bash
# Monitor arming state
ros2 topic echo /fmu/out/vehicle_status --field arming_state

# Manually arm if needed (arming_state should be 2)
ros2 service call /fmu/in/vehicle_command px4_msgs/srv/VehicleCommand \
  '{command: 400, param1: 1.0}'
```

3. **Restart the node:**
```bash
# Kill the current node
ros2 node kill /bioinspired_path_generator

# Restart it
ros2 run uav_planning bioinspired_path_generator
```

### Issue 3: Gazebo Won't Start

**Symptoms:**
```
[gazebo-1] process has died
Gazebo server error
```

**Solutions:**

1. **Check Gazebo installation:**
```bash
gazebo --version
which gazebo
```

2. **Try headless mode:**
```bash
ros2 launch uav_planning uav_simulation.launch.py headless:=true
```

3. **Clear Gazebo cache:**
```bash
rm -rf ~/.gazebo/models/.database_cache
```

4. **Check display settings (if using remote desktop):**
```bash
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1
```

5. **Install missing dependencies:**
```bash
sudo apt update
sudo apt install gazebo libgazebo-dev
```

### Issue 4: PX4 Connection Issues

**Symptoms:**
```
No px4_msgs available
Connection to PX4 failed
```

**Solutions:**

1. **Install px4_msgs:**
```bash
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd ~/ros2_ws
colcon build --packages-select px4_msgs
source install/setup.bash
```

2. **Check microXRCE DDS Agent:**
```bash
# Check if agent is running
ps aux | grep micro-xrce

# Start agent manually if needed
micro-xrce-dds-agent udp4 -p 8888
```

3. **Verify PX4 SITL is running:**
```bash
ps aux | grep px4
```

4. **Check DDS communication:**
```bash
ros2 topic list | grep fmu
```

### Issue 5: Import Errors

**Symptoms:**
```python
ModuleNotFoundError: No module named 'px4_msgs'
ImportError: cannot import name 'ButterflyExplorer'
```

**Solutions:**

1. **Check Python path:**
```bash
python3 -c "import sys; print('\n'.join(sys.path))"
```

2. **Reinstall package in development mode:**
```bash
cd ~/ros2_ws/src/uav-agricultural-sensing/src/uav_planning
pip3 install -e .
```

3. **Source workspace again:**
```bash
source ~/ros2_ws/install/setup.bash
```

4. **Check for missing dependencies:**
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Issue 6: Performance Issues

**Symptoms:**
- Simulation runs very slowly
- High CPU usage
- Delayed responses

**Solutions:**

1. **Reduce update rates:**
```bash
ros2 param set /bioinspired_path_generator trajectory_publish_rate 5.0
ros2 param set /bioinspired_path_generator waypoint_generation_rate 0.5
```

2. **Run headless simulation:**
```bash
ros2 launch uav_planning uav_simulation.launch.py headless:=true
```

3. **Close unnecessary applications:**
```bash
# Monitor system resources
htop

# Close browser, IDEs, etc.
```

4. **Adjust Gazebo physics:**
```bash
# In Gazebo GUI: World -> Physics -> Real Time Factor = 0.5
```

### Issue 7: Parameter Not Found

**Symptoms:**
```
Parameter 'alpha' not found
Could not get parameter
```

**Solutions:**

1. **Check if node is running:**
```bash
ros2 node list | grep bioinspired
```

2. **List available parameters:**
```bash
ros2 param list /bioinspired_path_generator
```

3. **Use correct parameter names:**
```bash
# Correct parameter names:
ros2 param get /bioinspired_path_generator alpha
ros2 param get /bioinspired_path_generator x_min
ros2 param get /bioinspired_path_generator velocity
```

### Issue 8: Build Errors

**Symptoms:**
```
CMake Error: Could not find a package configuration file
Package 'xxx' not found
```

**Solutions:**

1. **Install missing dependencies:**
```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

2. **Clean build:**
```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select uav_planning
```

3. **Check Python dependencies:**
```bash
pip3 install numpy matplotlib scipy
```

## 🔧 Advanced Troubleshooting

### Debug Mode

Enable debug logging for more detailed output:

```bash
export ROS_LOG_LEVEL=DEBUG
ros2 run uav_planning bioinspired_path_generator
```

### Network Issues

If running distributed systems:

```bash
# Check DDS discovery
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Test communication
ros2 topic pub /test std_msgs/String "data: hello"
ros2 topic echo /test
```

### Memory Issues

Monitor memory usage:

```bash
# Check memory usage
free -h
ps aux --sort=-%mem | head

# Reduce memory usage
export GAZEBO_MODEL_DATABASE_URI=""
```

### Log Analysis

Analyze ROS 2 logs:

```bash
# View recent logs
ros2 log view

# Filter logs by node
ros2 log view | grep bioinspired

# Check for specific errors
ros2 log view | grep -i error
```

## 🔍 Diagnostic Commands

### System Information
```bash
# OS and ROS version
lsb_release -a
echo $ROS_DISTRO

# Hardware info
lscpu | grep "Model name"
free -h
df -h
```

### ROS 2 Health Check
```bash
# Check ROS 2 daemon
ros2 daemon status

# Test basic ROS 2 functionality
ros2 topic pub /test std_msgs/String "data: test" --once
ros2 topic echo /test --once

# Check node lifecycle
ros2 lifecycle nodes
```

### PX4 Health Check
```bash
# Check PX4 process
ps aux | grep px4

# Check PX4 logs
tail -f ~/PX4-Autopilot/build/px4_sitl_default/tmp/rootfs/fs/microsd/log/*.ulg

# Test PX4 commander
cd ~/PX4-Autopilot
make px4_sitl shell
# In PX4 shell: commander status
```

## 📊 Performance Monitoring

### Real-time Monitoring
```bash
# Monitor topic frequencies
ros2 topic hz /fmu/in/trajectory_setpoint
ros2 topic hz /bioinspired_path_generator/current_waypoint

# Monitor system resources
htop
iotop
```

### Profiling
```bash
# Time command execution
time ros2 run uav_planning bioinspired_path_generator

# Profile Python code
python3 -m cProfile -o profile.stats your_script.py
```

## 🆘 Getting Help

If none of these solutions work:

### Gather Information
Before asking for help, collect:

1. **System info:**
```bash
uname -a
lsb_release -a
echo $ROS_DISTRO
```

2. **Full error output:**
```bash
ros2 run uav_planning bioinspired_path_generator 2>&1 | tee error.log
```

3. **Environment variables:**
```bash
env | grep ROS
env | grep PX4
env | grep GAZEBO
```

### Report Issues

Create a GitHub issue with:
- Clear description of the problem
- Steps to reproduce
- Full error output
- System information
- What you've already tried

### Contact Support

- **Email**: riccardo.enrico97@gmail.com
- **GitHub Issues**: [Create an issue](https://github.com/your-username/uav-agricultural-sensing/issues)
- **Documentation**: Check other sections in [docs/](../README.md)

## 📚 Additional Resources

- [ROS 2 Troubleshooting](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [PX4 User Guide](https://docs.px4.io/main/en/)
- [Gazebo Classic Documentation](http://gazebosim.org/tutorials)
- [Ubuntu Community Help](https://help.ubuntu.com/)

---

*Troubleshooting guide last updated: January 5, 2025*
