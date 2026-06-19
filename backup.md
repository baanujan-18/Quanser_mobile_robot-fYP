# SD Card Recovery Guide
## If SD card corrupts — full recovery steps

### What you need
- New SD card (32GB or 64GB recommended)
- Windows laptop with MobaXterm
- Raspberry Pi Imager (already installed)
- Internet connection

---

### Step 1 — Buy new SD card
- 32GB or 64GB
- Class 10 / A1 speed rating recommended

---

### Step 2 — Flash Ubuntu 22.04 onto new SD card
Open Raspberry Pi Imager on Windows:
- Choose OS → Other general-purpose OS → Ubuntu → Ubuntu Server 22.04 LTS 64-bit
- Choose Storage → select your new SD card
- Click gear icon (Advanced Settings):
  - Hostname: `raspberrypi`
  - Enable SSH: Yes
  - Username: `kobuki`
  - Password: (same as your original password)
  - WiFi SSID: your lab WiFi name
  - WiFi Password: your lab WiFi password
  - WiFi Country: LK
- Click Write → wait for it to finish

---

### Step 3 — Boot Pi and SSH in
- Insert new SD card into Raspberry Pi 4
- Connect Ethernet cable to laptop
- Power on Pi
- Wait 3 minutes
- Open MobaXterm and SSH in:
```bash
ssh kobuki@pi
```

---

### Step 4 — Update Ubuntu and install essentials
```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y curl wget git build-essential python3-pip
sudo reboot
```

---

### Step 5 — Install ROS 2 Humble
```bash
# Set locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL \
  https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install -y ros-humble-ros-base
sudo apt install -y python3-colcon-common-extensions \
  python3-rosdep python3-argcomplete

# Configure environment
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

---

### Step 6 — Install all required ROS 2 packages
```bash
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-depthimage-to-laserscan \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-teleop-twist-keyboard \
  ros-humble-laser-filters \
  ros-humble-rtabmap-ros \
  ros-humble-freenect-camera \
  libfreenect0.5 libfreenect-bin \
  libfreenect-dev freenect
```

---

### Step 7 — Clone GitHub repo
```bash
cd /home/kobuki
git clone https://github.com/baanujan-18/Quanser_mobile_robot-fYP.git
```

---

### Step 8 — Restore all FYP files
```bash
cp -r ~/Quanser_mobile_robot-fYP/fyp_ws        ~/
cp -r ~/Quanser_mobile_robot-fYP/Ros2-KinectV1 ~/
cp -r ~/Quanser_mobile_robot-fYP/maps           ~/
cp    ~/Quanser_mobile_robot-fYP/fix_kinect_camera_info.py ~/
```

---

### Step 9 — Build Kobuki driver
```bash
cd /home/kobuki/fyp_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
echo 'source ~/fyp_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

---

### Step 10 — Build Kinect driver
```bash
cd /home/kobuki/Ros2-KinectV1
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-skip ros2_kinect_mic_node
echo 'source ~/Ros2-KinectV1/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

---

### Step 11 — Fix USB permissions for QBot 2
```bash
sudo chmod 666 /dev/ttyUSB0
sudo usermod -aG dialout kobuki
sudo reboot
```

---

### Step 12 — Done! Verify everything works
```bash
# Check ROS 2:
ros2 run demo_nodes_cpp talker

# Check Kobuki:
ros2 launch kobuki_ros kobuki_node-launch.py

# Check Kinect:
sudo modprobe -r gspca_kinect
ros2 run ros2_kinect_depth depth_node

# Check topics:
ros2 topic hz /kinect/depth/image_raw   # expect 30 Hz
ros2 topic hz /odom                     # expect data
```

---

### Recovery time estimate
| Task | Time |
|---|---|
| Flash new SD card | 10 min |
| Update Ubuntu + install essentials | 10 min |
| Install ROS 2 Humble | 15 min |
| Install ROS 2 packages | 10 min |
| git clone + restore files | 2 min |
| colcon build | 10 min |
| **Total** | **~57 minutes** |

---

### Important files saved on GitHub
| File/Folder | Location on Pi |
|---|---|
| fyp_ws | ~/fyp_ws |
| Ros2-KinectV1 | ~/Ros2-KinectV1 |
| maps | ~/maps |
| fix_kinect_camera_info.py | ~/ |
| nav2_params.yaml | ~/fyp_ws/src/qbot2_bringup/config/ |
| slam_params.yaml | ~/fyp_ws/src/qbot2_bringup/config/ |
| laser_filter.yaml | ~/fyp_ws/src/qbot2_bringup/config/ |
| qbot2.urdf | ~/fyp_ws/src/qbot2_description/urdf/ |

---

*FYP — Baanujan E/20/030 | QBot 2 + Kinect v1 + ROS 2 Humble*
*Supervisor: Dr. D.H.S. Maithripala*
