# Desktop/Simulation-specific bash history (CPU-only)
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
# PX4 Simulation commands
cd /workspaces/PX4-Autopilot
make px4_sitl gz_x500_custom
PX4_SYS_AUTOSTART=4150 PX4_GZ_MODEL_POSE="0,0,0,0,0,0" PX4_GZ_MODEL=x500_custom ./build/px4_sitl_default/bin/px4
gz sim -v4 -r
plotjuggler
# Gazebo commands
gz model --list
gz topic --list
gz topic --echo /world/default/model/x500_custom/joint_state
# Build and development
colcon test --packages-select uav_planning
colcon build --packages-select uav_planning --symlink-install
rosdep install --from-paths src --ignore-src -r -y
# Python development
python3 -m pip install --upgrade pip
jupyter notebook --ip=0.0.0.0 --port=8888 --no-browser --allow-root
