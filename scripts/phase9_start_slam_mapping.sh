#!/bin/bash

# ============================================================

# Phase 9 - Start SLAM Toolbox Mapping for QBot 2

# Project: Quanser QBot 2 Autonomous Navigation

# Student: Baanujan | E/20/030

#

# IMPORTANT:

# This script is mainly a command helper.

# Phase 9 needs 6 separate MobaXterm terminals running at the same time.

# Copy each TAB command and run it in a separate terminal.

# ============================================================

echo "============================================================"
echo "PHASE 9 - SLAM TOOLBOX MAPPING COMMANDS"
echo "============================================================"
echo ""
echo "Open 6 MobaXterm SSH tabs and run the commands below."
echo ""

echo "------------------------------------------------------------"
echo "TAB 1 - Kobuki Driver"
echo "------------------------------------------------------------"
cat << 'EOF'
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash
ros2 launch kobuki_ros kobuki_node-launch.py
EOF

echo ""
echo "------------------------------------------------------------"
echo "TAB 2 - Kinect Depth Node"
echo "------------------------------------------------------------"
cat << 'EOF'
cd ~/Ros2-KinectV1
source /opt/ros/humble/setup.bash
source install/setup.bash
sudo modprobe -r gspca_kinect
ros2 run ros2_kinect_depth depth_node
EOF

echo ""
echo "------------------------------------------------------------"
echo "TAB 3 - Camera Info Fixer"
echo "------------------------------------------------------------"
cat << 'EOF'
source /opt/ros/humble/setup.bash
python3 ~/fix_kinect_camera_info.py
EOF

echo ""
echo "If using the script from this GitHub repository location, use:"
cat << 'EOF'
source /opt/ros/humble/setup.bash
python3 ros2_ws/src/qbot2_perception/scripts/fix_kinect_camera_info.py
EOF

echo ""
echo "------------------------------------------------------------"
echo "TAB 4 - Depth to LaserScan"
echo "------------------------------------------------------------"
cat << 'EOF'
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash
ros2 launch qbot2_bringup depth_to_scan.launch.py
EOF

echo ""
echo "------------------------------------------------------------"
echo "TAB 5 - Robot State Publisher"
echo "------------------------------------------------------------"
cat << 'EOF'
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash
ros2 launch qbot2_bringup robot_state.launch.py
EOF

echo ""
echo "------------------------------------------------------------"
echo "TAB 6 - SLAM Toolbox"
echo "------------------------------------------------------------"
cat << 'EOF'
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false
EOF

echo ""
echo "============================================================"
echo "VERIFY REQUIRED TOPICS"
echo "============================================================"
cat << 'EOF'
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash
ros2 topic list | grep -E "odom|scan|tf|map"
EOF

echo ""
echo "Expected topics:"
cat << 'EOF'
/odom
/scan
/tf
/tf_static
/map
/map_metadata
EOF

echo ""
echo "============================================================"
echo "DRIVE ROBOT FOR MAPPING"
echo "============================================================"
echo "IMPORTANT: Kobuki uses /commands/velocity, not /cmd_vel."
echo ""
cat << 'EOF'
sudo apt install -y ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/commands/velocity
EOF

echo ""
echo "============================================================"
echo "OPEN RVIZ2 TO VIEW MAP"
echo "============================================================"
cat << 'EOF'
ssh -X [kobuki@raspberrypi.local](mailto:kobuki@raspberrypi.local)
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash
rviz2
EOF

echo ""
echo "In RViz2:"
echo "1. Set Fixed Frame to map"
echo "2. Add /map as Map display"
echo "3. Add /scan as LaserScan display"
echo "4. Add TF display"
echo "5. Drive slowly and check the map building"
echo ""

echo "============================================================"
echo "SAVE MAP"
echo "============================================================"
cat << 'EOF'
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab_map
EOF

echo ""
echo "Expected saved files:"
cat << 'EOF'
~/maps/lab_map.pgm
~/maps/lab_map.yaml
EOF

echo ""
echo "============================================================"
echo "PHASE 9 RESULT"
echo "============================================================"
echo "If /odom, /scan, /tf, /map are working and lab_map files are saved,"
echo "then Phase 9 SLAM Toolbox mapping is completed successfully."
echo "============================================================"

