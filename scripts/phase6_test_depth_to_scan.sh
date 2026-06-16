#!/bin/bash

# ============================================================

# Phase 6 - Test Kinect Depth to LaserScan Conversion

# Project: Quanser QBot 2 Autonomous Navigation

# Student: Baanujan | E/20/030

#

# This script documents the working Phase 6 setup.

# Kinect v1 depth data is converted into /scan using

# depthimage_to_laserscan.

# ============================================================

echo "============================================================"
echo "PHASE 6 - DEPTH TO LASERSCAN TEST"
echo "============================================================"

echo ""
echo "Step 1 - Install depthimage_to_laserscan package"
echo "Run this if the package is not already installed:"
cat << 'EOF'
sudo apt install -y ros-humble-depthimage-to-laserscan
EOF

echo ""
echo "Step 2 - Start Kinect depth node in TAB 1"
cat << 'EOF'
cd ~/Ros2-KinectV1
source /opt/ros/humble/setup.bash
source install/setup.bash
sudo modprobe -r gspca_kinect
ros2 run ros2_kinect_depth depth_node
EOF

echo ""
echo "Expected Kinect topics:"
cat << 'EOF'
/kinect/depth/image_raw
/kinect/depth/camera_info
EOF

echo ""
echo "Step 3 - Start camera info fixer in TAB 2"
echo "During original testing, the script was run from home directory:"
cat << 'EOF'
source /opt/ros/humble/setup.bash
python3 ~/fix_kinect_camera_info.py
EOF

echo ""
echo "If using the script stored inside this GitHub repository, run:"
cat << 'EOF'
source /opt/ros/humble/setup.bash
python3 ros2_ws/src/qbot2_perception/scripts/fix_kinect_camera_info.py
EOF

echo ""
echo "Expected fixed camera info topic:"
cat << 'EOF'
/kinect/depth/camera_info_fixed
EOF

echo ""
echo "Step 4 - Start depth_to_scan launch file in TAB 3"
cat << 'EOF'
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash
ros2 launch qbot2_bringup depth_to_scan.launch.py
EOF

echo ""
echo "Step 5 - Check /scan topic in TAB 4"
cat << 'EOF'
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash
ros2 topic list | grep scan
ros2 topic hz /scan
ros2 topic echo /scan --once
EOF

echo ""
echo "============================================================"
echo "EXPECTED DATA FLOW"
echo "============================================================"
cat << 'EOF'
Kinect depth node
↓
/kinect/depth/image_raw

Camera info fixer
↓
/kinect/depth/camera_info_fixed

depthimage_to_laserscan
↓
/scan
EOF

echo ""
echo "============================================================"
echo "EXPECTED RESULT"
echo "============================================================"
echo "If /scan appears and publishes data, Phase 6 is successful."
echo ""
echo "Important topics:"
cat << 'EOF'
/kinect/depth/image_raw
/kinect/depth/camera_info
/kinect/depth/camera_info_fixed
/scan
EOF

echo ""
echo "============================================================"
echo "PHASE 6 COMPLETE CONDITION"
echo "============================================================"
echo "Phase 6 is completed when:"
echo "1. Kinect depth node publishes /kinect/depth/image_raw"
echo "2. Camera info fixer publishes /kinect/depth/camera_info_fixed"
echo "3. depthimage_to_laserscan publishes /scan"
echo "============================================================"

