# Project Dependencies

This document lists the main hardware, software, ROS 2 packages, external packages, and custom packages used in this project up to Phase 9.

---

## **System Information**

```text
Operating System: Ubuntu 22.04 LTS
ROS Version: ROS 2 Humble
Main Computer: Raspberry Pi 4
Robot Platform: Quanser QBot 2 / Kobuki Mobile Base
Depth Sensor: Xbox Kinect v1
Username: kobuki
```

---

## **Main Hardware Dependencies**

* Raspberry Pi 4
* Quanser QBot 2
* Kobuki mobile base
* Xbox Kinect v1
* Windows laptop for SSH access
* USB cable for QBot 2 connection
* Kinect power adapter
* Wi-Fi network for remote access
* MobaXterm for SSH and X11 forwarding

---

## **Ubuntu Packages**

The following Ubuntu packages were used during setup:

```bash
sudo apt install -y curl wget git build-essential python3-pip
sudo apt install -y locales software-properties-common
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-argcomplete
```

---

## **ROS 2 Installation**

ROS 2 Humble base installation:

```bash
sudo apt install -y ros-humble-ros-base
```

ROS 2 environment source command:

```bash
source /opt/ros/humble/setup.bash
```

This command was also added to `.bashrc`:

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```

---

## **ROS 2 Packages Used**

The following ROS 2 packages were used in this project:

```text
ros-humble-ros-base
ros-humble-depthimage-to-laserscan
ros-humble-robot-state-publisher
ros-humble-joint-state-publisher
ros-humble-slam-toolbox
ros-humble-nav2-map-server
ros-humble-teleop-twist-keyboard
```

Install command:

```bash
sudo apt install -y ros-humble-depthimage-to-laserscan
sudo apt install -y ros-humble-robot-state-publisher ros-humble-joint-state-publisher
sudo apt install -y ros-humble-slam-toolbox
sudo apt install -y ros-humble-nav2-map-server
sudo apt install -y ros-humble-teleop-twist-keyboard
```

---

## **Kinect v1 Dependencies**

Kinect v1 was used with `libfreenect`.

```bash
sudo apt install -y libfreenect0.5 libfreenect-bin libfreenect-dev freenect
```

Kinect USB fix command:

```bash
sudo modprobe -r gspca_kinect
```

This command was needed to avoid:

```text
LIBUSB_ERROR_BUSY
```

---

## **External GitHub Packages Used**

The following external packages were cloned and used during the project.

### **Kobuki / QBot 2 Driver Packages**

```text
kobuki_ros
kobuki_ros_interfaces
kobuki_core
velocity_smoother
ecl_tools
```

These packages were cloned into:

```text
~/fyp_ws/src/
```

---

### **Kinect v1 ROS 2 Package**

```text
KinectV1-Ros2
```

This package was cloned into:

```text
~/Ros2-KinectV1/
```

Branch used:

```text
ros2-humble
```

Build command used:

```bash
colcon build --symlink-install --packages-skip ros2_kinect_mic_node
```

The microphone package was skipped because it was not required for SLAM and caused build issues.

---

## **Important Note About External Packages**

The external packages listed above should not be uploaded fully into this GitHub repository.

They are third-party packages and should only be mentioned as dependencies.

Do not upload these folders into this repository:

```text
kobuki_ros
kobuki_ros_interfaces
kobuki_core
velocity_smoother
ecl_tools
KinectV1-Ros2
```

Only custom project files are stored in this repository.

---

## **Custom ROS 2 Packages in This Repository**

The following custom packages were created for this project:

```text
qbot2_bringup
qbot2_description
qbot2_perception
qbot2_slam
qbot2_navigation
```

---

## **qbot2_bringup**

Purpose:

* Stores launch files for the QBot 2 system.
* Launches depth-to-laserscan conversion.
* Launches Robot State Publisher.

Important files:

```text
ros2_ws/src/qbot2_bringup/launch/depth_to_scan.launch.py
ros2_ws/src/qbot2_bringup/launch/robot_state.launch.py
```

---

## **qbot2_description**

Purpose:

* Stores the QBot 2 URDF robot model.
* Defines important robot frames.
* Adds Kinect depth frame for SLAM.

Important file:

```text
ros2_ws/src/qbot2_description/urdf/qbot2.urdf
```

Important frames:

```text
base_footprint
base_link
camera_link
kinect_depth_frame
```

---

## **qbot2_perception**

Purpose:

* Stores perception-related helper scripts.
* Contains the camera info fixer script.

Important file:

```text
ros2_ws/src/qbot2_perception/scripts/fix_kinect_camera_info.py
```

The script subscribes to:

```text
/kinect/depth/camera_info
```

and publishes fixed camera info to:

```text
/kinect/depth/camera_info_fixed
```

---

## **qbot2_slam**

Purpose:

* Stores SLAM-related documentation and commands.
* Documents the working SLAM Toolbox mapping pipeline.

Important file:

```text
ros2_ws/src/qbot2_slam/commands/slam_toolbox_mapping_commands.md
```

---

## **qbot2_navigation**

Purpose:

* Reserved for Phase 10 Nav2 autonomous navigation.
* Will store Nav2 configuration files after testing.

Important planned file:

```text
ros2_ws/src/qbot2_navigation/config/nav2_params.yaml
```

Note:

```text
nav2_params.yaml will be added after Phase 10 testing.
```

---

## **Important ROS 2 Topics**

### **Kobuki / QBot 2 Topics**

```text
/odom
/commands/velocity
/joint_states
```

Important note:

```text
The robot uses /commands/velocity, not /cmd_vel.
```

Teleop remapping:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/commands/velocity
```

---

### **Kinect v1 Topics**

```text
/kinect/depth/image_raw
/kinect/depth/camera_info
```

---

### **Camera Info Fixer Topic**

```text
/kinect/depth/camera_info_fixed
```

---

### **Laser Scan Topic**

```text
/scan
```

---

### **TF Topics**

```text
/tf
/tf_static
```

---

### **SLAM Toolbox Topics**

```text
/map
/map_metadata
```

---

## **Working Sensor Pipeline**

```text
Kinect v1
   ↓
/kinect/depth/image_raw

Camera Info Fixer
   ↓
/kinect/depth/camera_info_fixed

depthimage_to_laserscan
   ↓
/scan

SLAM Toolbox
   ↓
/map
```

---

## **Working SLAM Inputs**

SLAM Toolbox uses:

```text
/odom
/scan
/tf
/tf_static
```

SLAM Toolbox produces:

```text
/map
/map_metadata
```

---

## **Saved Map Files**

The generated map was saved as:

```text
maps/lab_map.pgm
maps/lab_map.yaml
```

Original save command:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab_map
```

---

## **Current Completion Status**

Completed phases:

```text
Phase 0 - Project preparation
Phase 1 - Ubuntu setup
Phase 2 - Essential tools installation
Phase 3 - ROS 2 Humble setup
Phase 4 - Kobuki driver setup
Phase 5 - Kinect v1 setup
Phase 6 - Depth to LaserScan conversion
Phase 7 - Robot description using URDF
Phase 8 - Robot State Publisher
Phase 9 - SLAM Toolbox mapping
```

Next phase:

```text
Phase 10 - Autonomous navigation using Nav2
```

