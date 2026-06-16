#!/bin/bash

# ============================================================

# Phase 3 - Install ROS 2 Humble

# Project: Quanser QBot 2 Autonomous Navigation

# Student: Baanujan | E/20/030

#

# IMPORTANT:

# The repository command must use:

# dpkg --print-architecture

# There must be a space between dpkg and --print-architecture.

# ============================================================

echo "============================================================"
echo "PHASE 3 - INSTALL ROS 2 HUMBLE"
echo "============================================================"

echo ""
echo "Step 1 - Set locale"
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo ""
echo "Step 2 - Install software properties and enable universe repository"
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y

echo ""
echo "Step 3 - Add ROS 2 GPG key"
sudo apt update
sudo apt install -y curl

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key 
-o /usr/share/keyrings/ros-archive-keyring.gpg

echo ""
echo "Step 4 - Add ROS 2 Humble repository"
echo "IMPORTANT: The command below uses dpkg --print-architecture with a space."

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" 
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo ""
echo "Step 5 - Install ROS 2 Humble base"
sudo apt update
sudo apt install -y ros-humble-ros-base

echo ""
echo "Step 6 - Install ROS 2 build tools"
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-argcomplete

echo ""
echo "Step 7 - Source ROS 2 environment"
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash

echo ""
echo "============================================================"
echo "ROS 2 HUMBLE INSTALLATION COMPLETE"
echo "============================================================"

echo ""
echo "To test ROS 2, open two terminals."

echo ""
echo "TAB 1 - Talker:"
cat << 'EOF'
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
EOF

echo ""
echo "TAB 2 - Listener:"
cat << 'EOF'
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp listener
EOF

echo ""
echo "Expected listener output:"
cat << 'EOF'
[INFO] I heard: Hello World: 1
[INFO] I heard: Hello World: 2
EOF

echo ""
echo "If Hello World messages appear, Phase 3 is successful."
echo "============================================================"

