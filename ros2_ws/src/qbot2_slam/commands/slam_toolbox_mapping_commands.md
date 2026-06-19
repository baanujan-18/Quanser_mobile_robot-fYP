# SLAM Toolbox Mapping Commands

## Phase

Phase 9 - Install SLAM Toolbox and Build the Map

## Goal

The goal of this phase was to build a 2D map using SLAM Toolbox with the Quanser QBot 2 robot.

The SLAM system used:

* Kobuki odometry from `/odom`
* Kinect v1 depth data converted into `/scan`
* TF frames from Robot State Publisher
* SLAM Toolbox for map generation

---

## Install SLAM Toolbox

```bash
sudo apt install -y ros-humble-slam-toolbox
```

---

## Launch Full Mapping System

The full mapping system was launched using six MobaXterm tabs.

---

## Tab 1 - Kobuki Driver

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

sudo chmod 666 /dev/ttyUSB0

ros2 launch kobuki_node kobuki_node-launch.py
```

Purpose:

* Starts the Kobuki/QBot 2 driver
* Publishes odometry on `/odom`
* Receives velocity commands on `/commands/velocity`

---

## Tab 2 - Kinect Depth Node

```bash
cd ~/Ros2-KinectV1
source /opt/ros/humble/setup.bash
source install/setup.bash

sudo modprobe -r gspca_kinect

ros2 run ros2_kinect_depth depth_node
```

Purpose:

* Starts Kinect v1 depth node
* Publishes depth image data on `/kinect/depth/image_raw`

Important note:

```text
sudo modprobe -r gspca_kinect
```

This command is needed to avoid the `LIBUSB_ERROR_BUSY` problem.

---

## Tab 3 - Camera Info Fixer

During actual testing, the script was run from the home directory:

```bash
source /opt/ros/humble/setup.bash

python3 ~/fix_kinect_camera_info.py
```

In this GitHub repository, the same script is stored at:

```text
ros2_ws/src/qbot2_perception/scripts/fix_kinect_camera_info.py
```

So it can also be run from the repository location using:

```bash
source /opt/ros/humble/setup.bash
python3 ros2_ws/src/qbot2_perception/scripts/fix_kinect_camera_info.py
```

Purpose:

* Subscribes to `/kinect/depth/camera_info`
* Publishes corrected camera info to `/kinect/depth/camera_info_fixed`

---

## Tab 4 - Depth to LaserScan

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 run depthimage_to_laserscan depthimage_to_laserscan_node --ros-args \
-r depth:=/kinect/depth/image_raw \
-r depth_camera_info:=/kinect/depth/camera_info_fixed \
-r scan:=/scan \
-p scan_height:=10 \
-p range_min:=0.45 \
-p range_max:=4.0 \
-p output_frame:=kinect_depth_frame
```

Purpose:

* Converts Kinect depth image into fake laser scan
* Publishes `/scan`

---

## Tab 5 - Robot State Publisher

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 launch qbot2_bringup robot_state.launch.py
```

Purpose:

* Publishes robot TF frames from the QBot 2 URDF
* Provides the transform between `base_footprint`, `base_link`, and `kinect_depth_frame`

---

## Tab 5 - Laser Filter

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 run laser_filters scan_to_scan_filter_chain \
--ros-args \
--params-file /home/kobuki/fyp_ws/src/qbot2_bringup/config/laser_filter.yaml \
-r scan:=/scan \
-r scan_filtered:=/scan_filtered
```
---

## Tab 6 - SLAM Toolbox

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false
```

Purpose:

* Starts SLAM Toolbox
* Uses `/scan`, `/odom`, and TF data
* Publishes `/map`

---

## Verify Required Topics

In a new terminal, run:

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash
ros2 topic list | grep -E "odom|scan|tf|map"
```

Expected topics:

```text
/odom
/scan
/tf
/tf_static
/map
/map_metadata
```

---

## Drive Robot for Mapping

Install teleop if needed:

```bash
sudo apt install -y ros-humble-teleop-twist-keyboard
```

Run teleop with remapping:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/commands/velocity
```

Important note:

The Kobuki base uses:

```text
/commands/velocity
```

not:

```text
/cmd_vel
```

If the remapping is not used, the teleop window may run but the robot will not move.

---

## View Map in RViz2

Open RViz2 using X11 forwarding:

```bash
ssh -X kobuki@raspberrypi.local
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash
rviz2
```

In RViz2:

1. Set Fixed Frame to `map`
2. Add `/map` as Map display
3. Add `/scan` as LaserScan display
4. Add TF display
5. Drive the robot slowly and observe the map building

---

## Save the Map

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab_map
```

Expected saved files:

```text
lab_map.pgm
lab_map.yaml
```

---

## Final Result

SLAM Toolbox successfully generated a 2D map using Kinect v1 depth data converted into `/scan`.

The saved map is ready for Phase 10 autonomous navigation using Nav2.

