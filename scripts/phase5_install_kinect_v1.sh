#!/bin/bash

# ============================================================

# Phase 5 - Install and Test Kinect v1 for ROS 2 Humble

# Project: Quanser QBot 2 Autonomous Navigation

# Student: Baanujan | E/20/030

#

# This script documents the working Kinect v1 setup.

# KinectV1-Ros2 was used instead of OpenNI2.

# ============================================================

echo "============================================================"
echo "PHASE 5 - KINECT v1 SETUP"
echo "============================================================"

echo ""
echo "Step 1 - Install libfreenect packages"
sudo apt update
sudo apt install -y libfreenect0.5 libfreenect-bin libfreenect-dev freenect

echo ""
echo "Step 2 - Check Kinect hardware connection"
echo "Run this command and check for Xbox NUI Camera / Audio / Motor:"
lsusb

echo ""
echo "Step 3 - Clone KinectV1-Ros2 package"
mkdir -p ~/Ros2-KinectV1
cd ~/Ros2-KinectV1

if [ ! -d ".git" ]; then
git clone -b ros2-humble https://github.com/SriharshaShesham/KinectV1-Ros2.git .
else
echo "KinectV1-Ros2 repository already exists."
fi

echo ""
echo "Step 4 - Build KinectV1-Ros2 package"
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-skip ros2_kinect_mic_node

echo ""
echo "Step 5 - Source Kinect workspace"
source ~/Ros2-KinectV1/install/setup.bash

echo "source ~/Ros2-KinectV1/install/setup.bash" >> ~/.bashrc

echo ""
echo "Step 6 - Load Kinect USB fix"
echo "This avoids LIBUSB_ERROR_BUSY."
sudo modprobe -r gspca_kinect

echo ""
echo "============================================================"
echo "KINECT TEST COMMANDS"
echo "============================================================"
echo ""
echo "Run this in one terminal:"
cat << 'EOF'
cd ~/Ros2-KinectV1
source /opt/ros/humble/setup.bash
source install/setup.bash
sudo modprobe -r gspca_kinect
ros2 run ros2_kinect_depth depth_node
EOF

echo ""
echo "Run this in another terminal:"
cat << 'EOF'
source /opt/ros/humble/setup.bash
source ~/Ros2-KinectV1/install/setup.bash
ros2 topic list
ros2 topic hz /kinect/depth/image_raw
EOF

echo ""
echo "Expected working topics:"
cat << 'EOF'
/kinect/depth/image_raw
/kinect/depth/camera_info
EOF

echo ""
echo "Expected depth rate:"
echo "average rate: around 30 Hz"

echo ""
echo "============================================================"
echo "PHASE 5 RESULT"
echo "============================================================"
echo "If /kinect/depth/image_raw publishes around 30 Hz,"
echo "then Kinect v1 setup is completed successfully."
echo "============================================================"

