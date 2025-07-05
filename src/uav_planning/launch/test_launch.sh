#!/bin/bash
# Quick test script to verify the launch files work.
# This script tests launching the path planner without actually starting the full simulation.

# Source ROS 2 and workspace
source /opt/ros/jazzy/setup.bash
source /workspaces/uav-agri-sensing/install/setup.bash

echo "=== Testing Path Planner Launch File ==="
echo "Available launch files:"
ls -la /workspaces/uav-agri-sensing/install/uav_planning/share/uav_planning/launch/

echo ""
echo "=== Testing launch file syntax ==="
python3 -c "
import sys
import os
sys.path.insert(0, '/workspaces/uav-agri-sensing/install/uav_planning/share/uav_planning/launch/')

# Test if the files can be compiled
try:
    compile(open('/workspaces/uav-agri-sensing/install/uav_planning/share/uav_planning/launch/path_planner.launch.py').read(), 'path_planner.launch.py', 'exec')
    print('✓ path_planner.launch.py syntax OK')
except Exception as e:
    print(f'✗ path_planner.launch.py error: {e}')

try:
    compile(open('/workspaces/uav-agri-sensing/install/uav_planning/share/uav_planning/launch/uav_simulation.launch.py').read(), 'uav_simulation.launch.py', 'exec')
    print('✓ uav_simulation.launch.py syntax OK')
except Exception as e:
    print(f'✗ uav_simulation.launch.py error: {e}')
"

echo ""
echo "=== Quick launch test (dry run) ==="
echo "To start the path planner with microXRCE DDS:"
echo "ros2 launch uav_planning path_planner.launch.py"
echo ""
echo "To start the complete simulation:"
echo "ros2 launch uav_planning uav_simulation.launch.py"
echo ""
echo "To start with custom parameters:"
echo "ros2 launch uav_planning path_planner.launch.py x_min:=-50.0 x_max:=50.0 velocity:=12.0"
