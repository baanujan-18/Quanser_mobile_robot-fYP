# Setup Status

## **Project**

**Quanser QBot 2 Autonomous Navigation System**

---

## **Current Status**

The project setup has been completed successfully up to:

```text id="jqibmd"
Phase 9 - SLAM Toolbox Mapping
```

The robot can now build a 2D map using Kinect v1 depth data converted into laser scan data.

---

## **Completed Phases**

### **Phase 0 - Project Preparation**

Completed.

Main work:

* Selected Quanser QBot 2 as the robot platform.
* Selected Raspberry Pi 4 as the onboard computer.
* Selected Xbox Kinect v1 as the depth sensor.
* Planned ROS 2 Humble-based navigation system.

---

### **Phase 1 - Ubuntu 22.04 Setup**

Completed.

Main work:

* Installed Ubuntu 22.04 LTS on Raspberry Pi 4.
* Configured the Raspberry Pi username as:

```text id="p0ah4z"
kobuki
```

* Connected to the Raspberry Pi from Windows using MobaXterm SSH.

---

### **Phase 2 - Essential Tools Installation**

Completed.

Main tools installed:

```bash id="nc7wkv"
sudo apt update
sudo apt upgrade -y
sudo apt install -y curl wget git build-essential python3-pip
```

---

### **Phase 3 - ROS 2 Humble Setup**

Completed.

Main work:

* Added ROS 2 Humble repository.
* Installed ROS 2 Humble base.
* Installed ROS 2 build tools.
* Tested ROS 2 using talker and listener nodes.

Important corrected command:

```text id="8rdz51"
dpkg --print-architecture
```

There must be a space between `dpkg` and `--print-architecture`.

---

### **Phase 4 - Kobuki / QBot 2 Driver Setup**

Completed.

Main work:

* Created ROS 2 workspace:

```text id="88gx0d"
~/fyp_ws
```

* Cloned required Kobuki packages.
* Installed dependencies using `rosdep`.
* Built the workspace using `colcon`.
* Connected QBot 2 through USB.
* Launched Kobuki driver.

Important working topics:

```text id="ectmt4"
/odom
/commands/velocity
/joint_states
```

Important note:

```text id="3412h1"
QBot 2 uses /commands/velocity, not /cmd_vel.
```

---

### **Phase 5 - Kinect v1 Setup**

Completed.

Main work:

* Installed `libfreenect`.
* Cloned and built `KinectV1-Ros2`.
* Used Kinect v1 instead of OpenNI2.
* Applied Kinect USB fix:

```bash id="v72ekn"
sudo modprobe -r gspca_kinect
```

Working Kinect topics:

```text id="4j4ndk"
/kinect/depth/image_raw
/kinect/depth/camera_info
```

---

### **Phase 6 - Depth to LaserScan Conversion**

Completed.

Main work:

* Installed `depthimage_to_laserscan`.
* Created camera info fixer script.
* Published fixed camera info.
* Converted Kinect depth data into `/scan`.

Working pipeline:

```text id="2a3zyf"
Kinect depth image
        ↓
/kinect/depth/image_raw
        ↓
Camera info fixer
        ↓
/kinect/depth/camera_info_fixed
        ↓
depthimage_to_laserscan
        ↓
/scan
```

Important output topic:

```text id="8z0zkg"
/scan
```

---

### **Phase 7 - Robot Description using URDF**

Completed.

Main work:

* Created `qbot2_description` package.
* Created `qbot2.urdf`.
* Added robot base frames.
* Added Kinect depth frame.

Important frames:

```text id="gmoyw8"
base_footprint
base_link
camera_link
kinect_depth_frame
```

---

### **Phase 8 - Robot State Publisher**

Completed.

Main work:

* Created `robot_state.launch.py`.
* Launched Robot State Publisher.
* Published TF frames from the URDF model.

Working TF topics:

```text id="6z0n73"
/tf
/tf_static
```

---

### **Phase 9 - SLAM Toolbox Mapping**

Completed successfully.

Main work:

* Installed SLAM Toolbox.
* Started full mapping system.
* Verified `/odom`, `/scan`, `/tf`, and `/map`.
* Drove the robot manually using teleop.
* Viewed the map in RViz2.
* Saved the generated map.

Required topics verified:

```text id="tk7sus"
/odom
/scan
/tf
/tf_static
/map
/map_metadata
```

Map saved as:

```text id="8r0dwj"
lab_map.pgm
lab_map.yaml
```

Original map save command:

```bash id="v1y463"
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab_map
```

---

## **Current Working System**

The current working system is:

```text id="d58w8l"
QBot 2 / Kobuki base
        ↓
/odom

Kinect v1 depth sensor
        ↓
/kinect/depth/image_raw

Camera info fixer
        ↓
/kinect/depth/camera_info_fixed

Depth to LaserScan
        ↓
/scan

Robot State Publisher
        ↓
/tf and /tf_static

SLAM Toolbox
        ↓
/map
```

---

## **Current Result**

The robot can:

* Start the Kobuki base driver.
* Publish odometry.
* Start the Kinect v1 depth node.
* Convert Kinect depth data into `/scan`.
* Publish TF frames.
* Run SLAM Toolbox.
* Build a 2D map.
* Save the generated map.

---

## **Pending Phase**

### **Phase 10 - Autonomous Navigation using Nav2**

Not completed yet.

Planned tasks:

* Install Nav2.
* Create Nav2 configuration file.
* Load the saved map.
* Start localization.
* Open RViz2.
* Set initial robot pose.
* Send navigation goal.
* Test autonomous movement.

Planned map files:

```text id="cc0qg4"
maps/lab_map.pgm
maps/lab_map.yaml
```

Planned Nav2 config file:

```text id="boz9cp"
ros2_ws/src/qbot2_navigation/config/nav2_params.yaml
```

Important note:

```text id="s2w7kp"
nav2_params.yaml will be added after Phase 10 testing.
```

---

## **Overall Completion**

```text id="p1nizr"
Completed: Phase 0 to Phase 9
Pending:   Phase 10 and final autonomous navigation testing
```

---

## **Next Immediate Task**

Start Phase 10:

```text id="v5raw3"
Autonomous Navigation using Nav2 with the saved SLAM map
```
