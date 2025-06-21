# UAV-specific bash history
colcon build --symlink-install
ros2 launch uav_planning uav_planning_launch.py
ros2 topic list
ros2 topic echo /uav/position
ros2 service list
ros2 node list
MicroXRCEAgent udp4 -p 8888
ros2 run uav_planning uav_planning_node
htop
tree /workspaces
cd /workspaces/uav_control
source install/setup.bash
