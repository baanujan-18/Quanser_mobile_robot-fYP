#!/bin/bash

# ============================================================

# Phase 4 - Install and Test Kobuki Driver for QBot 2

# Project: Quanser QBot 2 Autonomous Navigation

# Student: Baanujan | E/20/030

#

# This script documents the working Kobuki/QBot 2 setup.

# The QBot 2 uses the Kobuki mobile base.

# ============================================================

echo "============================================================"
echo "PHASE 4 - KOBUKI / QBOT 2 DRIVER SETUP"
echo "============================================================"

echo ""
echo "Step 1 - Create ROS 2 workspace"
mkdir -p ~/fyp_ws/src
cd ~/fyp_ws/src

echo ""
echo "Step 2 - Clone Kobuki related packages"

if [ ! -d "kobuki_ros" ]; then
git clone https://github.com/kobuki-base/kobuki_ros.git
else
echo "kobuki_ros already exists."
fi

if [ ! -d "kobuki_ros_interfaces" ]; then
git clone https://github.com/kobuki-base/kobuki_ros_interfaces.git
else
echo "kobuki_ros_interfaces already exists."
fi

if [ ! -d "kobuki_core" ]; then
git clone https://github.com/kobuki-base/kobuki_core.git
else
echo "kobuki_core already exists."
fi

if [ ! -d "velocity_smoother" ]; then
git clone https://github.com/kobuki-base/velocity_smoother.git
else
echo "velocity_smoother already exists."
fi

if [ ! -d "ecl_tools" ]; then
git clone https://github.com/stonier/ecl_tools.git
else
echo "ecl_tools already exists."
fi

echo ""
echo "Step 3 - Install dependencies using rosdep"
cd ~/fyp_ws

echo ""
echo "If rosdep was never initialized before, run this once manually:"
echo "sudo rosdep init"
echo ""

rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo ""
echo "Step 4 - Build workspace"
source /opt/ros/humble/setup.bash
colcon build --symlink-install

echo ""
echo "Step 5 - Source workspace"
source ~/fyp_ws/install/setup.bash

echo "source ~/fyp_ws/install/setup.bash" >> ~/.bashrc

echo ""
echo "Step 6 - Check Kobuki packages"
ros2 pkg list | grep kobuki

echo ""
echo "============================================================"
echo "QBOT 2 PHYSICAL TEST COMMANDS"
echo "============================================================"
echo ""
echo "Connect QBot 2 to Raspberry Pi using USB."
echo ""
echo "Then run:"
cat << 'EOF'
ls /dev/ttyUSB*
EOF

echo ""
echo "Expected output:"
cat << 'EOF'
/dev/ttyUSB0
EOF

echo ""
echo "Give temporary USB permission:"
cat << 'EOF'
sudo chmod 666 /dev/ttyUSB0
EOF

echo ""
echo "Add user to dialout group:"
cat << 'EOF'
sudo usermod -aG dialout kobuki
sudo reboot
EOF

echo ""
echo "After reboot, launch Kobuki driver:"
cat << 'EOF'
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash
ros2 launch kobuki_ros kobuki_node-launch.py
EOF

echo ""
echo "In another terminal, check topics:"
cat << 'EOF'
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash
ros2 topic list
EOF

echo ""
echo "Expected working topics:"
cat << 'EOF'
/odom
/commands/velocity
/joint_states
EOF

echo ""
echo "============================================================"
echo "IMPORTANT NOTE"
echo "============================================================"
echo "The Kobuki base uses /commands/velocity for movement."
echo "It does not use the normal /cmd_vel topic directly."
echo ""
echo "For teleoperation, use:"
cat << 'EOF'
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/commands/velocity
EOF

echo ""
echo "============================================================"
echo "PHASE 4 COMPLETE CONDITION"
echo "============================================================"
echo "Phase 4 is successful when /odom publishes and"
echo "/commands/velocity and /joint_states are visible."
echo "============================================================"

