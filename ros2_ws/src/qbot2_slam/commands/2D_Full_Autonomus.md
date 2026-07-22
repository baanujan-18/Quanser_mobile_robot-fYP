# Quanser QBot 2 — Complete 2D Mapping and Full Autonomous Exploration Guide

> **Operating-mode warning**
>
> This document is only for the original **2D autonomous system** based on the
> depth-only Kinect node, `depthimage_to_laserscan`, SLAM Toolbox, Nav2, and
> `explore_lite`.
>
> Do not run the RGB-D `kinect_ros2_node`, RTAB-Map, RGB-D point-cloud nodes,
> keyboard teleoperation, or a temporary camera `static_transform_publisher`
> while using this 2D autonomous mode. The RGB-D workspace
> `~/kinect_rgb_ws` remains separate and does not need to be deleted.
>
> Only one Kinect driver and only one motion-command source should run at a
> time.

## Phase

Phase 9 - Install SLAM Toolbox and Build the Map

## Goal

The goal of this phase was to build a 2D map using SLAM Toolbox with the Quanser QBot 2 robot.

The SLAM system used:

* Kobuki odometry from `/odom`
* Kinect v1 depth data converted into `/scan`
* Filtered laser scan data from `/scan_filtered`
* TF frames from Robot State Publisher
* SLAM Toolbox for map generation

---

## Important Terminal Note

When testing continuous ROS 2 topics using commands such as:

```bash
ros2 topic hz /odom
```

stop the command using:

```text
Ctrl+C
```

Do not use:

```text
Ctrl+Z
```

`Ctrl+Z` pauses the command and leaves it as a stopped background job. This can cause Fast DDS shared-memory errors such as:

```text
RTPS_TRANSPORT_SHM Error
Failed init_port
open_and_lock_file failed
```

To remove stopped jobs, run:

```bash
jobs
kill $(jobs -p)
```

---

## Install SLAM Toolbox

```bash
sudo apt install -y ros-humble-slam-toolbox
```

---

# Launch Full Mapping System

The full mapping system was launched using seven MobaXterm tabs.

Each tab should be started in order.

After starting each tab, perform the corresponding test before continuing to the next tab.

---

# Tab 1 - Kobuki Driver

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

sudo chmod 666 /dev/ttyUSB0

ros2 launch kobuki_node kobuki_node-launch.py
```

## Purpose

* Starts the Kobuki/QBot 2 driver
* Publishes odometry on `/odom`
* Receives velocity commands on `/commands/velocity`

## Test Tab 1

Open a separate diagnostic terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash
```

Check that the Kobuki node is running:

```bash
ros2 node list | grep -i kobuki
```

Check that the odometry topic exists:

```bash
ros2 topic list | grep odom
```

Expected:

```text
/commands/reset_odometry
/odom
```

Check the odometry frequency:

```bash
ros2 topic hz /odom
```

Expected result:

```text
average rate: approximately 20 Hz
```

Stop the command using `Ctrl+C`.

Check one odometry message:

```bash
ros2 topic echo /odom --once
```

Check the odometry frame names:

```bash
ros2 topic echo /odom --once | grep -E "frame_id|child_frame_id"
```

Expected:

```text
frame_id: odom
child_frame_id: base_footprint
```

Check the velocity command topic:

```bash
ros2 topic info /commands/velocity -v
```

Expected:

* Topic type: `geometry_msgs/msg/Twist`
* At least one subscriber from the Kobuki driver

## Tab 1 Pass Condition

Tab 1 is working when:

* `/odom` exists
* `/odom` publishes continuously
* The frequency is approximately 20 Hz
* `/commands/velocity` has a Kobuki subscriber

---

# Tab 2 - Kinect Depth Node

```bash
cd ~/Ros2-KinectV1
source /opt/ros/humble/setup.bash
source install/setup.bash

sudo modprobe -r gspca_kinect

ros2 run ros2_kinect_depth depth_node
```

## Purpose

* Starts the Kinect v1 depth node
* Publishes depth image data on `/kinect/depth/image_raw`
* Publishes camera calibration data on `/kinect/depth/camera_info`

## Important Note

```bash
sudo modprobe -r gspca_kinect
```

This command is required to avoid the following problem:

```text
LIBUSB_ERROR_BUSY
```

## Test Tab 2

Check the Kinect topics:

```bash
ros2 topic list | grep kinect
```

Expected topics include:

```text
/kinect/depth/camera_info
/kinect/depth/image_raw
```

Check the depth image frequency:

```bash
ros2 topic hz /kinect/depth/image_raw
```

Expected result:

```text
approximately 30 Hz
```

Stop the command using `Ctrl+C`.

Check the depth image header:

```bash
ros2 topic echo /kinect/depth/image_raw --once --field header
```

Check the image properties:

```bash
ros2 topic echo /kinect/depth/image_raw --once | grep -E "height:|width:|encoding:"
```

Expected values are approximately:

```text
height: 480
width: 640
```

The exact encoding depends on the Kinect driver.

## Tab 2 Pass Condition

Tab 2 is working when:

* `/kinect/depth/image_raw` exists
* The depth image publishes continuously
* The frequency is approximately 30 Hz
* The image has valid width, height, and encoding values

---

# Tab 3 - Camera Info Fixer

During actual testing, the script was run from the home directory:

```bash
source /opt/ros/humble/setup.bash

python3 ~/fix_kinect_camera_info.py
```

In this GitHub repository, the same script is stored at:

```text
ros2_ws/src/qbot2_perception/scripts/fix_kinect_camera_info.py
```

It can also be run from the repository location:

```bash
source /opt/ros/humble/setup.bash

python3 ros2_ws/src/qbot2_perception/scripts/fix_kinect_camera_info.py
```

## Purpose

* Subscribes to `/kinect/depth/camera_info`
* Publishes corrected camera information to `/kinect/depth/camera_info_fixed`
* Provides usable intrinsic camera calibration values for `depthimage_to_laserscan`

## Test Tab 3

Check that the corrected camera information topic exists:

```bash
ros2 topic list | grep camera_info_fixed
```

Expected:

```text
/kinect/depth/camera_info_fixed
```

Check the publication frequency:

```bash
ros2 topic hz /kinect/depth/camera_info_fixed
```

The topic should publish continuously.

Stop the command using `Ctrl+C`.

Check one corrected camera information message:

```bash
ros2 topic echo /kinect/depth/camera_info_fixed --once
```

Expected values include:

```text
height: 480
width: 640
distortion_model: plumb_bob
```

The `k` matrix should contain valid intrinsic calibration values, for example:

```text
k:
- 525.0
- 0.0
- 319.5
- 0.0
- 525.0
- 239.5
- 0.0
- 0.0
- 1.0
```

The `p` matrix should also contain valid values.

Check the camera frame:

```bash
ros2 topic echo /kinect/depth/camera_info_fixed --once --field header
```

Expected frame:

```text
kinect_depth_optical_frame
```

Check the topic connections:

```bash
ros2 topic info /kinect/depth/camera_info_fixed -v
```

Expected:

* One publisher from the camera information fixer
* One subscriber from `depthimage_to_laserscan`, after Tab 4 is started

## Tab 3 Pass Condition

Tab 3 is working when:

* `/kinect/depth/camera_info_fixed` exists
* Messages are published continuously
* The width and height are valid
* The `k` and `p` matrices are not empty or all zeros
* The frame ID is valid

---

# Tab 4 - Depth to LaserScan

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

## Purpose

* Converts the Kinect depth image into laser scan data
* Publishes the generated scan on `/scan`
* Uses `kinect_depth_frame` as the scan frame

## Test Tab 4

Check that the scan topic exists:

```bash
ros2 topic list | grep scan
```

Expected:

```text
/scan
```

After Tab 6 is started, the result should include:

```text
/scan
/scan_filtered
```

Check the raw scan frequency:

```bash
ros2 topic hz /scan
```

Expected result:

```text
approximately 30 Hz
```

Stop the command using `Ctrl+C`.

Check one scan message:

```bash
ros2 topic echo /scan --once
```

Expected fields include:

```text
frame_id: kinect_depth_frame
angle_min:
angle_max:
angle_increment:
range_min: 0.45
range_max: 4.0
ranges:
```

The `ranges` array should contain real distance measurements.

Some `inf` values are normal when no obstacle is detected.

Check only the scan header:

```bash
ros2 topic echo /scan --once --field header
```

Expected:

```text
frame_id: kinect_depth_frame
```

Check the scan topic connections:

```bash
ros2 topic info /scan -v
```

Expected:

* Publisher from `depthimage_to_laserscan`
* Subscriber from the laser filter after Tab 6 is started

## Tab 4 Pass Condition

Tab 4 is working when:

* `/scan` exists
* `/scan` publishes continuously
* `range_min` is approximately `0.45`
* `range_max` is approximately `4.0`
* The `ranges` array contains valid distance values
* The frame ID is `kinect_depth_frame`

---

# Tab 5 - Robot State Publisher

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 launch qbot2_bringup robot_state.launch.py
```

## Purpose

* Publishes the robot TF frames from the QBot 2 URDF
* Provides transforms between the robot base and Kinect sensor
* Publishes static and dynamic TF data

Important frames include:

```text
odom
base_footprint
base_link
kinect_depth_frame
kinect_depth_optical_frame
```

## Test Tab 5

Check that Robot State Publisher is running:

```bash
ros2 node list | grep robot_state_publisher
```

Expected:

```text
/robot_state_publisher
```

Check that the URDF is loaded:

```bash
ros2 param get /robot_state_publisher robot_description
```

Expected result:

A long URDF/XML description.

It should not return:

```text
Parameter not set
```

Check the transform between `base_footprint` and `base_link`:

```bash
ros2 run tf2_ros tf2_echo base_footprint base_link
```

Expected:

```text
Translation: [...]
Rotation: [...]
```

Stop using `Ctrl+C`.

Check the transform between the robot base and Kinect:

```bash
ros2 run tf2_ros tf2_echo base_footprint kinect_depth_frame
```

This must return continuous transform data.

Check the optical frame transform:

```bash
ros2 run tf2_ros tf2_echo kinect_depth_frame kinect_depth_optical_frame
```

This must also return continuous transform data.

Check the complete base-to-optical transform:

```bash
ros2 run tf2_ros tf2_echo base_footprint kinect_depth_optical_frame
```

Check odometry TF from the Kobuki driver:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

The required pre-SLAM TF chain is:

```text
odom
└── base_footprint
    └── base_link
        └── kinect_depth_frame
            └── kinect_depth_optical_frame
```

Intermediate frames are acceptable as long as all required transforms are connected.

## Tab 5 Pass Condition

Tab 5 is working when all these commands succeed:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint base_link
ros2 run tf2_ros tf2_echo base_footprint kinect_depth_frame
ros2 run tf2_ros tf2_echo kinect_depth_frame kinect_depth_optical_frame
```

---

# Tab 6 - Laser Filter

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 run laser_filters scan_to_scan_filter_chain \
--ros-args \
--params-file /home/kobuki/fyp_ws/src/qbot2_bringup/config/laser_filter.yaml \
-r scan:=/scan \
-r scan_filtered:=/scan_filtered
```

## Purpose

* Subscribes to the raw `/scan` topic
* Applies the configured laser scan filters
* Publishes cleaned data on `/scan_filtered`
* Reduces Kinect noise before SLAM processing

## Test Tab 6

Check that both scan topics exist:

```bash
ros2 topic list | grep scan
```

Expected:

```text
/scan
/scan_filtered
```

Check the filtered scan frequency:

```bash
ros2 topic hz /scan_filtered
```

The frequency should normally be close to the `/scan` frequency.

Stop the command using `Ctrl+C`.

Check one filtered scan message:

```bash
ros2 topic echo /scan_filtered --once
```

Expected fields include:

```text
frame_id: kinect_depth_frame
range_min: 0.45
range_max: 4.0
ranges:
```

The `ranges` array should contain valid distance measurements.

Check the filtered scan header:

```bash
ros2 topic echo /scan_filtered --once --field header
```

Expected:

```text
frame_id: kinect_depth_frame
```

Check the raw scan connections:

```bash
ros2 topic info /scan -v
```

Expected:

* One publisher from `depthimage_to_laserscan`
* One subscriber from `scan_to_scan_filter_chain`

Check the filtered scan connections:

```bash
ros2 topic info /scan_filtered -v
```

Expected:

* One publisher from `scan_to_scan_filter_chain`
* A SLAM Toolbox subscriber after Tab 7 is started, if SLAM is configured to use `/scan_filtered`

Check the laser filter node:

```bash
ros2 node list | grep -E "filter|scan"
```

## Tab 6 Pass Condition

Tab 6 is working when:

* `/scan_filtered` exists
* `/scan_filtered` publishes continuously
* The frame ID is `kinect_depth_frame`
* The ranges contain valid measurements
* The laser filter subscribes to `/scan`

---

# Tab 7 - SLAM Toolbox

## Important Scan Topic Decision

The default SLAM Toolbox launch file normally subscribes to:

```text
/scan
```

If SLAM Toolbox should use the filtered scan, remap `/scan` to `/scan_filtered`.

Recommended command:

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 launch slam_toolbox online_async_launch.py \
use_sim_time:=false \
slam_params_file:=/home/kobuki/fyp_ws/src/qbot2_bringup/config/slam_toolbox.yaml
```

The SLAM Toolbox parameter file should contain:

```yaml
slam_toolbox:
  ros__parameters:
    scan_topic: /scan_filtered
```

If no custom parameter file is used, launch the default configuration:

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false
```

In that case, SLAM Toolbox will normally use `/scan`.

## Purpose

* Starts SLAM Toolbox
* Uses odometry, laser scan, and TF data
* Publishes the 2D occupancy grid on `/map`
* Publishes the `map → odom` transform

## Test Tab 7

Check that the SLAM Toolbox node is running:

```bash
ros2 node list | grep slam
```

Expected a node similar to:

```text
/slam_toolbox
```

Check that SLAM subscribes to the correct scan topic:

```bash
ros2 node info /slam_toolbox
```

Under subscriptions, check for either:

```text
/scan
```

or:

```text
/scan_filtered
```

Check that the map topic exists:

```bash
ros2 topic list | grep map
```

Expected topics include:

```text
/map
/map_metadata
```

Check that the map is publishing:

```bash
ros2 topic echo /map --once
```

Expected fields include:

```text
header:
info:
data:
```

Check the map frequency:

```bash
ros2 topic hz /map
```

The map may publish at a lower rate than the laser scan.

Stop the command using `Ctrl+C`.

Check the SLAM transform:

```bash
ros2 run tf2_ros tf2_echo map odom
```

Expected continuous transform output.

Check the complete map-to-robot transform:

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

This must also return transform data.

## Tab 7 Pass Condition

SLAM Toolbox is working when:

* `/slam_toolbox` is running
* `/map` exists
* `/map_metadata` exists
* `/map` publishes messages
* `map → odom` exists
* `map → base_footprint` exists

The complete mapping TF chain should now be:

```text
map
└── odom
    └── base_footprint
        └── base_link
            └── kinect_depth_frame
                └── kinect_depth_optical_frame
```

---

# Verify All Required Topics

In a new terminal, run:

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 topic list | grep -E "odom|scan|tf|map"
```

Expected topics include:

```text
/odom
/scan
/scan_filtered
/tf
/tf_static
/map
/map_metadata
```

---

# Full System Diagnostic Checklist

Run these commands one by one.

## Check Kobuki Odometry

```bash
ros2 topic echo /odom --once
```

## Check Kinect Depth Image

```bash
ros2 topic echo /kinect/depth/image_raw --once --field header
```

## Check Corrected Camera Information

```bash
ros2 topic echo /kinect/depth/camera_info_fixed --once --field header
```

## Check Raw Laser Scan

```bash
ros2 topic echo /scan --once --field header
```

## Check Filtered Laser Scan

```bash
ros2 topic echo /scan_filtered --once --field header
```

## Check Odometry TF

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

## Check Kinect TF

```bash
ros2 run tf2_ros tf2_echo base_footprint kinect_depth_frame
```

## Check Kinect Optical TF

```bash
ros2 run tf2_ros tf2_echo kinect_depth_frame kinect_depth_optical_frame
```

## Check SLAM TF

```bash
ros2 run tf2_ros tf2_echo map odom
```

## Check Complete Map TF

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

---

# Drive Robot for Mapping

Install keyboard teleoperation if needed:

```bash
sudo apt install -y ros-humble-teleop-twist-keyboard
```

Run teleoperation with the required remapping:

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
--ros-args \
-r cmd_vel:=/commands/velocity
```

## Important Note

The Kobuki base receives movement commands on:

```text
/commands/velocity
```

It does not directly receive commands on:

```text
/cmd_vel
```

Without the remapping, the teleoperation node may run while the robot remains stationary.

## Safe Movement Test

Before moving the robot, place it on a clear floor area.

In another terminal, verify the command topic:

```bash
ros2 topic echo /commands/velocity
```

When a teleoperation key is pressed, velocity messages should appear.

Drive the robot slowly while mapping.

Recommended actions:

* Move forward slowly
* Rotate gradually
* Avoid fast turns
* Keep the Kinect facing walls and obstacles
* Revisit previously mapped areas to improve loop closure

---

# View Map in RViz2

RViz2 can be opened using X11 forwarding:

```bash
ssh -X kobuki@raspberrypi.local
```

Then run:

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

rviz2
```

In RViz2:

1. Set **Fixed Frame** to:

```text
map
```

2. Add a **Map** display.

3. Set the Map topic to:

```text
/map
```

4. Add a **LaserScan** display.

5. Use either:

```text
/scan
```

or preferably:

```text
/scan_filtered
```

6. Add a **TF** display.

7. Add a **RobotModel** display if required.

8. Drive the robot slowly and observe the map building.

---

# RViz2 Verification

The system is working correctly when:

* The occupancy map appears
* The laser scan aligns with walls
* The robot model moves correctly
* The map remains mostly stable during movement
* Previously mapped walls align when revisited

If the map rotates, jumps, or duplicates walls, check:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

and:

```bash
ros2 run tf2_ros tf2_echo base_footprint kinect_depth_frame
```

---

# Save the Map

Create the map directory:

```bash
mkdir -p ~/maps
```

Save the generated map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab_map
```

Expected saved files:

```text
lab_map.pgm
lab_map.yaml
```

Check that both files exist:

```bash
ls -lh ~/maps
```

Expected:

```text
lab_map.pgm
lab_map.yaml
```

Inspect the YAML file:

```bash
cat ~/maps/lab_map.yaml
```

Expected fields include:

```yaml
image: lab_map.pgm
mode: trinary
resolution: 0.05
origin:
negate: 0
occupied_thresh:
free_thresh:
```

---

# Download the Saved Map to the Laptop

From the laptop terminal, run:

```bash
scp kobuki@raspberrypi.local:~/maps/lab_map.pgm .
scp kobuki@raspberrypi.local:~/maps/lab_map.yaml .
```

Alternatively, save both files into a local folder:

```bash
mkdir -p qbot2_map
scp kobuki@raspberrypi.local:~/maps/lab_map.pgm qbot2_map/
scp kobuki@raspberrypi.local:~/maps/lab_map.yaml qbot2_map/
```

The two files must remain in the same directory because the YAML file refers to the PGM image.

---

# Troubleshooting

## Fast DDS Shared Memory Error

Example:

```text
RTPS_TRANSPORT_SHM Error
Failed init_port
open_and_lock_file failed
```

Cause:

* A previous ROS 2 command was stopped using `Ctrl+Z`
* Multiple stopped ROS processes are holding DDS resources

Fix:

```bash
jobs
kill $(jobs -p)
```

Then open a new terminal and source the workspace again.

Always stop ROS commands using:

```text
Ctrl+C
```

---

## `/scan` Exists but `ros2 topic hz /scan` Shows Nothing

First check one message:

```bash
ros2 topic echo /scan --once
```

If a valid message appears, the scan publisher is working.

Then clear stopped jobs:

```bash
jobs
kill $(jobs -p)
```

Open a new terminal and retry:

```bash
ros2 topic hz /scan
```

---

## `/map` Does Not Appear

Check that SLAM Toolbox is running:

```bash
ros2 node list | grep slam
```

Check the scan subscription:

```bash
ros2 node info /slam_toolbox
```

Check the required transforms:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

```bash
ros2 run tf2_ros tf2_echo base_footprint kinect_depth_frame
```

Check that the scan topic is publishing:

```bash
ros2 topic echo /scan --once
```

or:

```bash
ros2 topic echo /scan_filtered --once
```

---

## `map → odom` Transform Is Missing

The `map → odom` transform is published by SLAM Toolbox.

Check that SLAM Toolbox is active:

```bash
ros2 node list | grep slam
```

Check for map messages:

```bash
ros2 topic echo /map --once
```

Then retry:

```bash
ros2 run tf2_ros tf2_echo map odom
```

---

## Laser Scan Does Not Align with the Map

Check the scan frame:

```bash
ros2 topic echo /scan --once --field header
```

Expected:

```text
frame_id: kinect_depth_frame
```

Check the Kinect transform:

```bash
ros2 run tf2_ros tf2_echo base_footprint kinect_depth_frame
```

Check the optical transform:

```bash
ros2 run tf2_ros tf2_echo kinect_depth_frame kinect_depth_optical_frame
```

Incorrect Kinect position or rotation values in the URDF can cause map distortion.

---

# Final System Pass Criteria

The mapping system is ready when all the following tests succeed:

```bash
ros2 topic echo /odom --once
ros2 topic echo /kinect/depth/image_raw --once --field header
ros2 topic echo /kinect/depth/camera_info_fixed --once --field header
ros2 topic echo /scan --once --field header
ros2 topic echo /scan_filtered --once --field header
ros2 topic echo /map --once
```

The following TF checks must also succeed:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint base_link
ros2 run tf2_ros tf2_echo base_footprint kinect_depth_frame
ros2 run tf2_ros tf2_echo kinect_depth_frame kinect_depth_optical_frame
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_footprint
```

The complete required TF chain is:

```text
map
└── odom
    └── base_footprint
        └── base_link
            └── kinect_depth_frame
                └── kinect_depth_optical_frame
```

---

# Phase 10 — Complete 2D Full Autonomous Exploration

## Goal

The goal of this phase is to make the Quanser QBot 2:

- start from an unmapped environment;
- build a live 2D occupancy map using SLAM Toolbox;
- detect unexplored frontiers using `explore_lite`;
- send frontier goals to Nav2;
- plan and follow safe paths;
- move without keyboard control;
- update the map continuously while moving;
- save the final maze or laboratory map;
- later reuse the saved map for localization and shortest-path navigation.

A previously saved map is **not required** during live exploration. SLAM
Toolbox must remain active because it supplies:

```text
/map
/map_metadata
/map_updates
map → odom
```

The complete autonomous pipeline is:

```text
Kobuki wheel odometry: /odom
Kinect depth image: /kinect/depth/image_raw
Corrected calibration: /kinect/depth/camera_info_fixed
                 │
                 ▼
depthimage_to_laserscan
                 │
                 ▼
              /scan
                 │
                 ▼
            laser_filters
                 │
                 ▼
          /scan_filtered
                 │
                 ├──────────────► Nav2 local/global costmaps
                 │
                 ▼
           SLAM Toolbox
                 │
                 ├──────────────► live /map and /map_updates
                 └──────────────► map → odom
                                      │
                                      ▼
                                  explore_lite
                                      │
                                      ▼
                              /navigate_to_pose
                                      │
                                      ▼
                                     Nav2
                                      │
                              controller output
                                      │
                                      ▼
                            velocity_smoother
                                      │
                                      ▼
                                   /cmd_vel
                                      │
                                      ▼
                             cmd_vel_bridge.py
                                      │
                                      ▼
                          /commands/velocity
                                      │
                                      ▼
                                  Kobuki base
```

---

# 10.1 Keep the 2D and 3D Modes Separate

Before starting the 2D autonomous system, stop these nodes with `Ctrl+C`:

```text
RGB-D kinect_ros2_node
RTAB-Map
PointCloudXyzNode
RGB-D synchronization nodes
manual teleop_twist_keyboard
temporary static_transform_publisher
old test nodes left in other terminals
```

Check for conflicting nodes:

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 node list | grep -Ei \
"kinect_ros2|rtabmap|pointcloud|rgbd|teleop|static_transform"
```

For clean 2D mode, the command should return no relevant active nodes.

Check for stopped shell jobs:

```bash
jobs
```

If stopped jobs are listed:

```bash
kill $(jobs -p)
```

Reset ROS discovery only when stale nodes or topics remain:

```bash
ros2 daemon stop
sleep 2
ros2 daemon start
```

Always use:

```text
Ctrl+C
```

Never use:

```text
Ctrl+Z
```

---

# 10.2 Required Packages

Install the main packages if they are not already installed:

```bash
sudo apt update

sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-depthimage-to-laserscan \
  ros-humble-laser-filters \
  ros-humble-teleop-twist-keyboard \
  ros-humble-nav2-map-server
```

Verify important packages:

```bash
ros2 pkg list | grep -E \
"slam_toolbox|nav2_bringup|depthimage_to_laserscan|laser_filters"
```

`explore_lite` must already be built in `~/fyp_ws`.

Verify:

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 pkg list | grep explore_lite
```

---

# 10.3 Configuration Backups

Before changing any working configuration:

```bash
mkdir -p ~/fyp_ws/config_backups
```

```bash
cp ~/fyp_ws/src/qbot2_bringup/config/slam_toolbox.yaml \
~/fyp_ws/config_backups/slam_toolbox_$(date +%Y%m%d_%H%M%S).yaml
```

```bash
cp ~/fyp_ws/src/qbot2_bringup/config/laser_filter.yaml \
~/fyp_ws/config_backups/laser_filter_$(date +%Y%m%d_%H%M%S).yaml
```

```bash
cp ~/fyp_ws/src/qbot2_bringup/config/nav2_params.yaml \
~/fyp_ws/config_backups/nav2_params_$(date +%Y%m%d_%H%M%S).yaml
```

```bash
cp ~/fyp_ws/src/qbot2_bringup/config/explore_params.yaml \
~/fyp_ws/config_backups/explore_params_$(date +%Y%m%d_%H%M%S).yaml
```

Do not replace the complete Nav2 file with a short fragment. Nav2 requires
configuration for every launched server. Make small changes to the last complete
working file.

---

# 10.4 Recommended SLAM Toolbox Settings

Open:

```bash
nano ~/fyp_ws/src/qbot2_bringup/config/slam_toolbox.yaml
```

The complete file must retain all required SLAM Toolbox parameters. Confirm at
least these critical values:

```yaml
slam_toolbox:
  ros__parameters:
    use_sim_time: false

    odom_frame: odom
    map_frame: map
    base_frame: base_footprint

    scan_topic: /scan_filtered
    mode: mapping

    transform_publish_period: 0.05
    map_update_interval: 2.0
    throttle_scans: 2

    resolution: 0.05
    minimum_time_interval: 0.5
    transform_timeout: 1.0
    tf_buffer_duration: 30.0
```

Do not remove the solver, loop-closure, scan-matching, or occupancy-grid
parameters already present in the complete working file.

Validate the YAML:

```bash
python3 - <<'PY'
import yaml

path = "/home/kobuki/fyp_ws/src/qbot2_bringup/config/slam_toolbox.yaml"

with open(path, "r", encoding="utf-8") as file:
    config = yaml.safe_load(file)

params = config["slam_toolbox"]["ros__parameters"]

print("YAML OK:", path)
print("scan_topic:", params.get("scan_topic"))
print("odom_frame:", params.get("odom_frame"))
print("map_frame:", params.get("map_frame"))
print("base_frame:", params.get("base_frame"))
print("transform_publish_period:", params.get("transform_publish_period"))
print("map_update_interval:", params.get("map_update_interval"))
print("throttle_scans:", params.get("throttle_scans"))
PY
```

Required output should show:

```text
scan_topic: /scan_filtered
odom_frame: odom
map_frame: map
base_frame: base_footprint
transform_publish_period: 0.05
map_update_interval: 2.0
throttle_scans: 2
```

---

# 10.5 Recommended `explore_params.yaml`

Open:

```bash
nano ~/fyp_ws/src/qbot2_bringup/config/explore_params.yaml
```

Use this conservative Raspberry Pi configuration:

```yaml
/**:
  ros__parameters:
    robot_base_frame: base_footprint

    costmap_topic: map
    costmap_updates_topic: map_updates

    return_to_init: false
    visualize: false

    planner_frequency: 0.10
    progress_timeout: 180.0
    transform_tolerance: 1.0

    potential_scale: 3.0
    orientation_scale: 0.0
    gain_scale: 1.0

    min_frontier_size: 0.05
```

Save:

```text
Ctrl+O
Enter
Ctrl+X
```

Validate:

```bash
python3 - <<'PY'
import yaml

path = "/home/kobuki/fyp_ws/src/qbot2_bringup/config/explore_params.yaml"

with open(path, "r", encoding="utf-8") as file:
    config = yaml.safe_load(file)

params = config["/**"]["ros__parameters"]

print("YAML OK:", path)
print("robot_base_frame:", params.get("robot_base_frame"))
print("costmap_topic:", params.get("costmap_topic"))
print("planner_frequency:", params.get("planner_frequency"))
print("progress_timeout:", params.get("progress_timeout"))
print("transform_tolerance:", params.get("transform_tolerance"))
print("min_frontier_size:", params.get("min_frontier_size"))
PY
```

---

# 10.6 Critical Nav2 Configuration

Open the last complete working Nav2 file:

```bash
nano ~/fyp_ws/src/qbot2_bringup/config/nav2_params.yaml
```

## Important Humble YAML Type Rule

For costmap dimensions, use integers:

```yaml
width: 3
height: 3
```

Do not use:

```yaml
width: 3.0
height: 3.0
```

On the tested ROS 2 Humble system, decimal values caused parameter-type errors
because those costmap parameters were declared as integers.

## Behavior Tree Navigator

Confirm these values exist under `bt_navigator`:

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_footprint
    odom_topic: /odom

    bt_loop_duration: 100
    default_server_timeout: 1000
```

Retain the complete navigator plugin and behavior-tree configuration from the
working file.

## Controller Server

Confirm the complete controller section contains the following critical values:

```yaml
controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 5.0

    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.10
      movement_time_allowance: 30.0

    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      stateful: true
      xy_goal_tolerance: 0.20
      yaw_goal_tolerance: 0.25

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"

      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.10
      max_vel_y: 0.0
      max_vel_theta: 0.30

      min_speed_xy: 0.02
      max_speed_xy: 0.10
      min_speed_theta: 0.10

      acc_lim_x: 0.30
      acc_lim_y: 0.0
      acc_lim_theta: 0.50

      decel_lim_x: -0.40
      decel_lim_y: 0.0
      decel_lim_theta: -0.70

      transform_tolerance: 1.0

      critics:
        - "RotateToGoal"
        - "Oscillation"
        - "BaseObstacle"
        - "GoalAlign"
        - "PathAlign"
        - "PathDist"
        - "GoalDist"

      BaseObstacle.scale: 0.05
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.10
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.10
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0
```

The `critics` list is mandatory. Without it, Nav2 fails with:

```text
No critics defined for FollowPath
```

Retain the trajectory sampling parameters from the last complete working file.
On the Raspberry Pi, keep the sample counts conservative rather than increasing
them aggressively.

## Planner Server

Confirm:

```yaml
planner_server:
  ros__parameters:
    use_sim_time: false
    expected_planner_frequency: 1.0
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 1.0
      use_astar: true
      allow_unknown: true
```

`allow_unknown: true` is required for frontier goals that lie near the
free/unknown boundary.

## Local Costmap

Confirm that the complete local costmap keeps the tested frames and scan topic:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: false

      update_frequency: 3.0
      publish_frequency: 2.0

      global_frame: odom
      robot_base_frame: base_footprint
      rolling_window: true

      width: 3
      height: 3
      resolution: 0.05

      transform_tolerance: 1.0

      robot_radius: 0.22
      footprint_padding: 0.02

      plugins:
        - "obstacle_layer"
        - "inflation_layer"

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan

        scan:
          topic: /scan_filtered
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: LaserScan
          raytrace_max_range: 4.0
          raytrace_min_range: 0.45
          obstacle_max_range: 4.0
          obstacle_min_range: 0.45

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: true
        inflation_radius: 0.35
        cost_scaling_factor: 3.0

      always_send_full_costmap: true
```

## Global Costmap

The global costmap must follow the changing SLAM map. Confirm:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: false

      update_frequency: 0.5
      publish_frequency: 1.0

      global_frame: map
      robot_base_frame: base_footprint
      resolution: 0.05

      track_unknown_space: true
      transform_tolerance: 1.0

      robot_radius: 0.22
      footprint_padding: 0.02

      plugins:
        - "static_layer"
        - "obstacle_layer"
        - "inflation_layer"

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        enabled: true
        map_topic: /map
        map_subscribe_transient_local: true
        subscribe_to_updates: true

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan

        scan:
          topic: /scan_filtered
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: LaserScan
          raytrace_max_range: 4.0
          raytrace_min_range: 0.45
          obstacle_max_range: 4.0
          obstacle_min_range: 0.45

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: true
        inflation_radius: 0.35
        cost_scaling_factor: 3.0

      always_send_full_costmap: true
```

Do not configure the global costmap as a fixed small rolling window. During
online SLAM, the static layer must resize with `/map` and process `/map_updates`.

## Velocity Smoother

Confirm:

```yaml
velocity_smoother:
  ros__parameters:
    use_sim_time: false

    smoothing_frequency: 20.0
    feedback: "OPEN_LOOP"
    scale_velocities: false

    max_velocity: [0.10, 0.0, 0.30]
    min_velocity: [-0.10, 0.0, -0.30]

    max_accel: [0.30, 0.0, 0.50]
    max_decel: [-0.40, 0.0, -0.70]

    deadband_velocity: [0.015, 0.0, 0.06]

    velocity_timeout: 0.5
    odom_topic: /odom
```

The existing complete file may contain topic remappings that create this route:

```text
controller_server → /cmd_vel_nav
velocity_smoother → /cmd_vel
cmd_vel_bridge.py → /commands/velocity
Kobuki
```

Do not remove those remappings unless the route is deliberately redesigned.

## Validate the Complete Nav2 YAML

```bash
python3 - <<'PY'
import yaml

path = "/home/kobuki/fyp_ws/src/qbot2_bringup/config/nav2_params.yaml"

with open(path, "r", encoding="utf-8") as file:
    config = yaml.safe_load(file)

required = [
    "bt_navigator",
    "controller_server",
    "planner_server",
    "smoother_server",
    "behavior_server",
    "waypoint_follower",
    "local_costmap",
    "global_costmap",
    "velocity_smoother",
]

missing = [key for key in required if key not in config]

if missing:
    raise SystemExit("ERROR: Missing top-level sections: " + ", ".join(missing))

controller = config["controller_server"]["ros__parameters"]
follow_path = controller["FollowPath"]
planner = config["planner_server"]["ros__parameters"]["GridBased"]
local = config["local_costmap"]["local_costmap"]["ros__parameters"]
global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]
smoother = config["velocity_smoother"]["ros__parameters"]

print("YAML OK:", path)
print("Controller frequency:", controller.get("controller_frequency"))
print("Controller plugin:", follow_path.get("plugin"))
print("Critics:", follow_path.get("critics"))
print("Planner tolerance:", planner.get("tolerance"))
print("Allow unknown:", planner.get("allow_unknown"))
print("Local size:", local.get("width"), "x", local.get("height"))
print("Local size types:",
      type(local.get("width")).__name__,
      type(local.get("height")).__name__)
print("Global plugins:", global_costmap.get("plugins"))
print("Global static-layer updates:",
      global_costmap.get("static_layer", {}).get("subscribe_to_updates"))
print("Velocity smoother frequency:",
      smoother.get("smoothing_frequency"))

if not follow_path.get("critics"):
    raise SystemExit("ERROR: FollowPath critics list is missing")

if local.get("width") is not None and not isinstance(local["width"], int):
    raise SystemExit("ERROR: local_costmap width must be an integer")

if local.get("height") is not None and not isinstance(local["height"], int):
    raise SystemExit("ERROR: local_costmap height must be an integer")

if planner.get("allow_unknown") is not True:
    raise SystemExit("ERROR: GridBased allow_unknown must be true")

print("Critical Nav2 checks passed")
PY
```

---

# 10.7 Complete Startup Order

Use separate MobaXterm terminals and start them in this exact order:

```text
Tab 1  — Kobuki driver
Tab 2  — Original depth-only Kinect node
Tab 3  — Camera-info fixer
Tab 4  — Depth image to LaserScan
Tab 5  — Robot State Publisher
Tab 6  — Laser filter
Tab 7  — SLAM Toolbox
Tab 8  — Nav2
Tab 9  — Velocity bridge
Tab 10 — CPU monitor, optional
Tab 11 — explore_lite
```

Do not start Tab 8 until Tabs 1–7 pass.

Do not start Tab 11 until Tabs 1–9 pass the mandatory readiness checklist.

---

# Tab 1 — Kobuki Driver

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

sudo chmod 666 /dev/ttyUSB0

ros2 launch kobuki_node kobuki_node-launch.py
```

Test:

```bash
ros2 topic echo /odom --once --field header
```

```bash
ros2 topic hz /odom
```

Expected approximately:

```text
20 Hz
```

Stop the rate check with `Ctrl+C`.

Check command subscriber:

```bash
ros2 topic info /commands/velocity -v
```

Before the bridge starts, the Kobuki subscriber should be present even if the
publisher count is zero.

Pass condition:

```text
/odom publishes continuously
frame_id is odom
child_frame_id is base_footprint
/commands/velocity has the Kobuki subscriber
```

---

# Tab 2 — Original Depth-Only Kinect Driver

Do not start the RGB-D driver from `~/kinect_rgb_ws`.

Run:

```bash
cd ~/Ros2-KinectV1

source /opt/ros/humble/setup.bash
source install/setup.bash

sudo modprobe -r gspca_kinect

ros2 run ros2_kinect_depth depth_node
```

Test:

```bash
ros2 topic echo /kinect/depth/image_raw --once --field header
```

```bash
ros2 topic hz /kinect/depth/image_raw
```

Stop with `Ctrl+C`.

Pass condition:

```text
/kinect/depth/image_raw publishes continuously
/kinect/depth/camera_info exists
image width and height are valid
only one Kinect driver is active
```

Check the active publisher:

```bash
ros2 topic info /kinect/depth/image_raw -v
```

Expected publisher count:

```text
1
```

---

# Tab 3 — Camera-Info Fixer

```bash
source /opt/ros/humble/setup.bash

python3 ~/fix_kinect_camera_info.py
```

Test:

```bash
ros2 topic echo \
/kinect/depth/camera_info_fixed \
--once \
--field header
```

```bash
ros2 topic echo \
/kinect/depth/camera_info_fixed \
--once \
--field k
```

Pass condition:

```text
frame_id: kinect_depth_optical_frame
non-zero K matrix
continuous publication
```

---

# Tab 4 — Depth to LaserScan

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 run depthimage_to_laserscan \
depthimage_to_laserscan_node \
--ros-args \
-r depth:=/kinect/depth/image_raw \
-r depth_camera_info:=/kinect/depth/camera_info_fixed \
-r scan:=/scan \
-p scan_height:=10 \
-p range_min:=0.45 \
-p range_max:=4.0 \
-p output_frame:=kinect_depth_frame
```

Test:

```bash
ros2 topic echo /scan --once --field header
```

```bash
ros2 topic hz /scan
```

Stop with `Ctrl+C`.

Check key fields:

```bash
ros2 topic echo /scan --once | grep -E \
"frame_id:|range_min:|range_max:"
```

Pass condition:

```text
frame_id: kinect_depth_frame
range_min: approximately 0.45
range_max: approximately 4.0
ranges array contains valid values
```

Some `inf` values are normal.

---

# Tab 5 — Robot State Publisher

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 launch qbot2_bringup robot_state.launch.py
```

Test all required pre-SLAM transforms:

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
odom \
base_footprint
```

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
base_footprint \
base_link
```

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
base_footprint \
kinect_depth_frame
```

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
kinect_depth_frame \
kinect_depth_optical_frame
```

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
base_footprint \
kinect_depth_optical_frame
```

Pass condition:

```text
All transforms return valid translation and rotation data
No disconnected frame
No second static publisher for the same Kinect frame
```

Required chain:

```text
odom
└── base_footprint
    └── base_link
        └── kinect_depth_frame
            └── kinect_depth_optical_frame
```

---

# Tab 6 — Laser Filter

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 run laser_filters scan_to_scan_filter_chain \
--ros-args \
--params-file \
/home/kobuki/fyp_ws/src/qbot2_bringup/config/laser_filter.yaml \
-r scan:=/scan \
-r scan_filtered:=/scan_filtered
```

Test:

```bash
ros2 topic echo /scan_filtered --once --field header
```

```bash
ros2 topic hz /scan_filtered
```

Stop with `Ctrl+C`.

Check connections:

```bash
ros2 topic info /scan -v
```

```bash
ros2 topic info /scan_filtered -v
```

Pass condition:

```text
/scan has one depthimage_to_laserscan publisher
/scan has a laser-filter subscriber
/scan_filtered has one filter publisher
/scan_filtered publishes continuously
frame_id is kinect_depth_frame
```

---

# Tab 7 — SLAM Toolbox

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 launch slam_toolbox online_async_launch.py \
use_sim_time:=false \
slam_params_file:=\
/home/kobuki/fyp_ws/src/qbot2_bringup/config/slam_toolbox.yaml
```

Wait until SLAM Toolbox is fully running.

Test scan subscription:

```bash
ros2 node info /slam_toolbox
```

The subscriptions must include:

```text
/scan_filtered
```

Test map:

```bash
ros2 topic echo /map --once --field info
```

Test map updates:

```bash
ros2 topic echo /map_updates --once
```

The update topic may not publish until the map changes.

Test transforms:

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
map \
odom
```

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
map \
base_footprint
```

Pass condition:

```text
/slam_toolbox exists
/map publishes
/map_metadata exists
map → odom exists
map → base_footprint exists
/map grows when the robot or scene changes
```

Wait for a valid `/map` message before starting Nav2.

---

# Tab 8 — Nav2

Start Nav2 only after Tabs 1–7 pass.

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 launch nav2_bringup navigation_launch.py \
use_sim_time:=false \
autostart:=true \
params_file:=\
/home/kobuki/fyp_ws/src/qbot2_bringup/config/nav2_params.yaml
```

Do not start `explore_lite` yet.

## Nav2 lifecycle checks

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /smoother_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /waypoint_follower
ros2 lifecycle get /velocity_smoother
```

Every required node must show:

```text
active [3]
```

Check action servers:

```bash
ros2 action list | grep -E \
"navigate_to_pose|navigate_through_poses|follow_path|spin|backup|wait|drive_on_heading"
```

At minimum, verify:

```text
/navigate_to_pose
/follow_path
```

Check costmaps:

```bash
ros2 topic list | grep -E \
"global_costmap/costmap$|local_costmap/costmap$"
```

Test one message from each:

```bash
ros2 topic echo \
/local_costmap/costmap \
--once \
--field info
```

```bash
ros2 topic echo \
/global_costmap/costmap \
--once \
--field info
```

Pass condition:

```text
All required lifecycle nodes are active
/navigate_to_pose exists
/follow_path exists
local and global costmaps publish
no parameter type crash
no "No critics defined for FollowPath" error
```

---

# Tab 9 — Velocity Bridge

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

python3 ~/cmd_vel_bridge.py
```

The bridge must subscribe to:

```text
/cmd_vel
```

and publish to:

```text
/commands/velocity
```

Check:

```bash
ros2 node list | grep cmd_vel_bridge
```

```bash
ros2 node info /cmd_vel_bridge
```

```bash
ros2 topic info /commands/velocity -v
```

Required after the bridge starts:

```text
Publisher count: 1
Subscription count: 1
```

The publisher should be the bridge.

The subscriber should be the Kobuki driver.

Do not run keyboard teleoperation while the bridge and autonomous stack are
active.

---

# Tab 10 — CPU and Memory Monitor

Run:

```bash
top
```

Useful keys:

```text
1  show per-core CPU usage
M  sort by memory
P  sort by CPU
q  quit
```

High CPU load may cause:

```text
delayed TF
stale scan data
behavior-tree timeouts
controller-loop delays
rejected or aborted goals
costmap update delays
jerky wheel motion
```

For autonomous 2D mode, keep these stopped:

```text
RTAB-Map
rtabmap_viz
RViz2 on the Pi when not needed
RGB-D point-cloud processing
manual teleoperation
```

---

# 10.8 Mandatory Readiness Checklist

Do not start `explore_lite` until every section below passes.

## A. Sensor and map data

```bash
ros2 topic echo /odom --once --field header
```

```bash
ros2 topic echo \
/kinect/depth/image_raw \
--once \
--field header
```

```bash
ros2 topic echo \
/kinect/depth/camera_info_fixed \
--once \
--field header
```

```bash
ros2 topic echo /scan --once --field header
```

```bash
ros2 topic echo /scan_filtered --once --field header
```

```bash
ros2 topic echo /map --once --field info
```

Required:

```text
All commands return data
No empty frame IDs
No missing odometry
/scan_filtered uses kinect_depth_frame
/map has non-zero width and height
```

## B. Topic rates

Run each separately and stop with `Ctrl+C`:

```bash
ros2 topic hz /odom
```

```bash
ros2 topic hz /kinect/depth/image_raw
```

```bash
ros2 topic hz /scan_filtered
```

Expected approximate behavior:

```text
/odom                       about 20 Hz
Kinect depth                continuous, normally near the driver rate
/scan_filtered              continuous and close to /scan
/map                        slower than the scan
```

## C. TF checks

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
odom \
base_footprint
```

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
base_footprint \
kinect_depth_frame
```

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
kinect_depth_frame \
kinect_depth_optical_frame
```

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
map \
odom
```

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
map \
base_footprint
```

Required TF chain:

```text
map
└── odom
    └── base_footprint
        └── base_link
            └── kinect_depth_frame
                └── kinect_depth_optical_frame
```

TF timestamps must continue advancing. A stationary position can repeat, but a
timestamp that stops advancing for several seconds indicates a failed stream.

## D. Nav2 lifecycle

```bash
for node in \
controller_server \
planner_server \
smoother_server \
behavior_server \
bt_navigator \
waypoint_follower \
velocity_smoother
do
  echo "===== $node ====="
  ros2 lifecycle get "/$node"
done
```

All required nodes must return:

```text
active [3]
```

## E. Navigation actions

```bash
ros2 action list | grep -E \
"navigate_to_pose|follow_path|spin|backup|wait"
```

Required:

```text
/navigate_to_pose
/follow_path
```

## F. Velocity route

Check the full route:

```bash
ros2 node info /controller_server
```

```bash
ros2 node info /velocity_smoother
```

```bash
ros2 node info /cmd_vel_bridge
```

```bash
ros2 topic info /commands/velocity -v
```

Required final state:

```text
/commands/velocity
Publisher count: 1
Subscription count: 1
```

## G. No conflicting command source

```bash
ros2 topic info /commands/velocity -v
```

There must not be multiple publishers.

Also check:

```bash
ros2 node list | grep -Ei \
"teleop|keyboard|rtabmap|kinect_ros2"
```

No conflicting 3D or manual-driving node should be active.

---

# 10.9 Global Costmap Bounds Check

A known failure mode is:

```text
SLAM /map grows
global costmap remains at an old smaller size
frontier goal lies outside the global costmap
planner reports worldToMap or no-path errors
explore_lite blacklists the frontier
```

Compare `/map` and `/global_costmap/costmap`:

```bash
python3 - <<'PY'
import time
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

rclpy.init()
node = rclpy.create_node("map_costmap_bounds_check")

messages = {}

map_qos = QoSProfile(depth=1)
map_qos.reliability = ReliabilityPolicy.RELIABLE
map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

costmap_qos = QoSProfile(depth=1)
costmap_qos.reliability = ReliabilityPolicy.RELIABLE
costmap_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

node.create_subscription(
    OccupancyGrid,
    "/map",
    lambda msg: messages.__setitem__("map", msg),
    map_qos,
)

node.create_subscription(
    OccupancyGrid,
    "/global_costmap/costmap",
    lambda msg: messages.__setitem__("global", msg),
    costmap_qos,
)

deadline = time.time() + 15.0

while len(messages) < 2 and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.5)

def describe(name, msg):
    width_m = msg.info.width * msg.info.resolution
    height_m = msg.info.height * msg.info.resolution
    min_x = msg.info.origin.position.x
    min_y = msg.info.origin.position.y
    max_x = min_x + width_m
    max_y = min_y + height_m

    print(f"{name}:")
    print(f"  cells: {msg.info.width} x {msg.info.height}")
    print(f"  resolution: {msg.info.resolution}")
    print(f"  origin: ({min_x:.3f}, {min_y:.3f})")
    print(f"  bounds: x=[{min_x:.3f}, {max_x:.3f}] "
          f"y=[{min_y:.3f}, {max_y:.3f}]")

if "map" not in messages:
    print("ERROR: no /map message")
else:
    describe("/map", messages["map"])

if "global" not in messages:
    print("ERROR: no /global_costmap/costmap message")
else:
    describe("/global_costmap/costmap", messages["global"])

if "map" in messages and "global" in messages:
    m = messages["map"].info
    g = messages["global"].info

    map_max_x = m.origin.position.x + m.width * m.resolution
    map_max_y = m.origin.position.y + m.height * m.resolution
    global_max_x = g.origin.position.x + g.width * g.resolution
    global_max_y = g.origin.position.y + g.height * g.resolution

    covers = (
        g.origin.position.x <= m.origin.position.x + 1e-3
        and g.origin.position.y <= m.origin.position.y + 1e-3
        and global_max_x >= map_max_x - 1e-3
        and global_max_y >= map_max_y - 1e-3
    )

    print("Global costmap covers current map:", covers)

node.destroy_node()
rclpy.shutdown()
PY
```

Required:

```text
Global costmap covers current map: True
```

Small transient differences while updates are arriving may be acceptable, but
the global costmap must not remain permanently smaller than `/map`.

If it remains stale:

1. Stop `explore_lite`.
2. Check the global static-layer configuration.
3. Confirm `map_subscribe_transient_local: true`.
4. Confirm `subscribe_to_updates: true`.
5. Confirm the global costmap is not a small rolling window.
6. Restart Nav2 after `/map` is available.
7. Run the bounds check again.
8. Start `explore_lite` only after the global costmap covers the current map.

---

# Tab 11 — Start Autonomous Exploration

Start only after all readiness checks pass:

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 run explore_lite explore \
--ros-args \
--params-file \
/home/kobuki/fyp_ws/src/qbot2_bringup/config/explore_params.yaml
```

Expected messages may include:

```text
Waiting for costmap to become available, topic: map
Waiting to connect to move_base nav2 server
Connected to move_base nav2 server
Getting initial pose of the robot
found N frontiers
Sending goal
Goal ACCEPTED
```

The robot should:

```text
detect a frontier
send a Nav2 goal
plan a path
move toward the frontier
update the map
repeat
```

Do not send a manual RViz goal while testing `explore_lite`. The frontier
explorer generates its own goals.

Keep the emergency stop ready.

---

# 10.10 Exploration Debug Logging

For a diagnostic run:

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 run explore_lite explore \
--ros-args \
--params-file \
/home/kobuki/fyp_ws/src/qbot2_bringup/config/explore_params.yaml \
--log-level debug \
2>&1 | tee ~/explore_debug.log
```

Extract important messages:

```bash
grep -E \
"found [0-9]+ frontiers|Sending goal|Goal ACCEPTED|Goal aborted|Goal was REJECTED|black list|All frontiers|No frontiers" \
~/explore_debug.log
```

Monitor Nav2 separately:

```bash
ros2 action list | grep navigate_to_pose
```

```bash
ros2 topic echo /cmd_vel_nav
```

If `/cmd_vel_nav` is not part of the actual configured route, inspect:

```bash
ros2 node info /controller_server
```

and use the controller server's exact output topic.

---

# 10.11 Interpret Common Exploration Results

## `Goal ACCEPTED`

This confirms the Nav2 action server accepted the frontier goal.

It does not guarantee successful motion. Continue checking:

```text
global plan creation
controller output
velocity smoother output
bridge output
Kobuki motion
```

## `Goal aborted`

Check the Nav2 terminal for:

```text
no valid path
controller failed
progress checker failed
TF timeout
costmap outside bounds
behavior-tree timeout
```

## `Goal was REJECTED`

Usually indicates:

```text
bt_navigator not active
Nav2 lifecycle startup incomplete
action server unavailable
another navigation goal already active
```

Check:

```bash
ros2 lifecycle get /bt_navigator
```

Required:

```text
active [3]
```

## `All frontiers traversed/tried out, stopping`

This does **not always mean the map is complete**.

It can mean:

```text
frontiers were found
Nav2 rejected or aborted them
explore_lite blacklisted them
no non-blacklisted frontier remained
```

Check:

```bash
grep -E \
"found [0-9]+ frontiers|Goal ACCEPTED|Goal aborted|Goal was REJECTED|black list|All frontiers" \
~/explore_debug.log
```

Restarting `explore_lite` clears its in-memory blacklist, but the same failure
will return if the underlying Nav2, TF, costmap, or motion problem is not fixed.

## `No frontiers found`

Check map content:

```bash
python3 - <<'PY'
import time
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

rclpy.init()
node = rclpy.create_node("map_cell_check")
messages = []

qos = QoSProfile(depth=1)
qos.reliability = ReliabilityPolicy.RELIABLE
qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

node.create_subscription(
    OccupancyGrid,
    "/map",
    lambda msg: messages.append(msg),
    qos,
)

deadline = time.time() + 10.0

while not messages and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.5)

if not messages:
    print("ERROR: no /map message received")
else:
    msg = messages[0]
    data = msg.data

    unknown = sum(value == -1 for value in data)
    free = sum(value == 0 for value in data)
    occupied = sum(value >= 65 for value in data)

    print("Width:", msg.info.width)
    print("Height:", msg.info.height)
    print("Resolution:", msg.info.resolution)
    print("Free cells:", free)
    print("Unknown cells:", unknown)
    print("Occupied cells:", occupied)

node.destroy_node()
rclpy.shutdown()
PY
```

Exploration normally requires:

```text
Free cells > 0
Unknown cells > 0
```

---

# 10.12 Robot Does Not Move

Use this sequence instead of changing many parameters at once.

## Step 1 — Confirm an accepted goal

Check the explore terminal for:

```text
Goal ACCEPTED
```

## Step 2 — Check controller output

```bash
ros2 node info /controller_server
```

Find the exact velocity output topic.

If configured as `/cmd_vel_nav`:

```bash
ros2 topic echo /cmd_vel_nav
```

Interpretation:

```text
No messages
→ no active valid plan, controller problem, or goal already failed

Messages present
→ continue to velocity_smoother
```

## Step 3 — Check smoothed velocity

```bash
ros2 node info /velocity_smoother
```

Then:

```bash
ros2 topic echo /cmd_vel
```

Interpretation:

```text
controller output present but /cmd_vel empty
→ velocity_smoother or remapping problem
```

## Step 4 — Check bridge output

```bash
ros2 node info /cmd_vel_bridge
```

```bash
ros2 topic echo /commands/velocity
```

Interpretation:

```text
/cmd_vel present but /commands/velocity empty
→ bridge problem

/commands/velocity present but robot does not move
→ Kobuki driver, bumper, cliff sensor, motor power, wheel, or hardware safety issue
```

## Step 5 — Check odometry

```bash
ros2 topic hz /odom
```

If `/odom` is missing, restart the Kobuki driver before continuing.

## Step 6 — Exclusive direct motor test

Only perform this after stopping:

```text
explore_lite
Nav2
velocity bridge
all other velocity publishers
```

Confirm no publisher:

```bash
ros2 topic info /commands/velocity -v
```

The only remaining endpoint should be the Kobuki subscriber.

Then run:

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
--ros-args \
-r cmd_vel:=/commands/velocity
```

Test slowly in a clear area.

Stop with `Ctrl+C`.

Do not leave teleoperation running when restarting autonomy.

---

# 10.13 TF Extrapolation or Timeout

Examples:

```text
Lookup would require extrapolation into the future
Transform timeout
Timed out waiting for transform
```

Check:

```bash
ros2 topic hz /odom
```

```bash
ros2 topic hz /scan_filtered
```

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
map \
odom
```

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
map \
base_footprint
```

Verify these tolerances:

```yaml
FollowPath:
  transform_tolerance: 1.0
```

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      transform_tolerance: 1.0
```

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      transform_tolerance: 1.0
```

```yaml
explore_params.yaml:
  transform_tolerance: 1.0
```

Increasing tolerance cannot repair a TF publisher that has stopped. Restart the
failed odometry, sensor, SLAM, or Nav2 stage.

---

# 10.14 Nav2 Process Exists but Node Is Missing

A Linux process can exist even when the ROS node failed to join the graph.

Compare:

```bash
pgrep -af controller_server
```

with:

```bash
ros2 node list --no-daemon --spin-time 3 | grep controller_server
```

If the process exists but the node does not, stop the Nav2 terminal and remove
stale processes:

```bash
pkill -9 -f controller_server
pkill -9 -f planner_server
pkill -9 -f smoother_server
pkill -9 -f behavior_server
pkill -9 -f bt_navigator
pkill -9 -f waypoint_follower
pkill -9 -f velocity_smoother
pkill -9 -f lifecycle_manager_navigation
```

Reset discovery:

```bash
ros2 daemon stop
sleep 2
ros2 daemon start
```

Restart Nav2 only after `/map`, `/odom`, `/scan_filtered`, and all required TF
transforms are available.

---

# 10.15 Costmap and Planning Failures

## Planner reports goal outside costmap or `worldToMap` failure

Run the global costmap bounds check in Section 10.9.

Verify:

```yaml
static_layer:
  map_topic: /map
  map_subscribe_transient_local: true
  subscribe_to_updates: true
```

Verify the global costmap is not configured as a small rolling window.

Restart Nav2 after `/map` is available.

## Planner cannot create a path to a frontier

Check:

```yaml
GridBased:
  tolerance: 1.0
  allow_unknown: true
  use_astar: true
```

Check that the frontier lies inside the current global costmap.

Check that inflation does not close a narrow corridor.

## Local costmap shows no obstacle data

Check:

```bash
ros2 node info /local_costmap/local_costmap
```

Verify it subscribes to:

```text
/scan_filtered
```

Check the scan:

```bash
ros2 topic echo /scan_filtered --once
```

Check:

```text
frame_id: kinect_depth_frame
```

Check the sensor transform:

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
base_footprint \
kinect_depth_frame
```

---

# 10.16 Robot Passes Too Close to Obstacles

Start with:

```yaml
robot_radius: 0.22
footprint_padding: 0.02
inflation_radius: 0.35
cost_scaling_factor: 3.0
```

Increase the inflation radius gradually:

```text
0.35 → 0.38 → 0.40
```

Do not immediately use a large value in a narrow maze. Excessive inflation can
make corridors appear blocked.

Check that both local and global costmaps use `/scan_filtered`.

## Kinect safety limitation

The current LaserScan conversion has approximately:

```text
minimum range: 0.45 m
maximum configured range: 4.0 m
forward horizontal field of view: about 63 degrees
```

The robot may not detect:

```text
objects closer than 45 cm
objects beside the robot
objects behind the robot
very low objects outside the selected image rows
transparent or poorly reflecting surfaces
drop-offs that are not handled by the base safety sensors
```

No costmap setting can recover sensor data that does not exist.

---

# 10.17 Wheels Stutter or Motion Is Jerky

Verify:

```yaml
controller_frequency: 5.0

FollowPath:
  max_vel_x: 0.10
  max_vel_theta: 0.30
  acc_lim_theta: 0.50
  decel_lim_theta: -0.70

velocity_smoother:
  smoothing_frequency: 20.0
  deadband_velocity: [0.015, 0.0, 0.06]
```

Check CPU load:

```bash
top
```

Check for duplicate command publishers:

```bash
ros2 topic info /commands/velocity -v
```

Check for delayed odometry:

```bash
ros2 topic hz /odom
```

Check for repeated goal aborts and recovery behaviors in the Nav2 terminal.

---

# 10.18 Safe Exploration Procedure

Before motion:

1. Place the robot in a clear, level area.
2. Keep an emergency stop or power switch ready.
3. Remove cables from the driving path.
4. Avoid stairs and drop-offs.
5. Keep people outside the immediate robot path.
6. Confirm the Kobuki bumper and cliff safety functions.
7. Confirm `/commands/velocity` has only one publisher.
8. Confirm Nav2 costmaps are publishing.
9. Confirm the global costmap covers the current map.
10. Start `explore_lite` only after all checks pass.

During motion:

```text
Watch the robot continuously
Stop it if the scan or map freezes
Stop it if it approaches an undetected obstacle
Do not assume explore_lite has full side or rear awareness
Do not leave the robot unattended
```

---

# 10.19 View the Autonomous Run in RViz2

Prefer running RViz2 on a laptop rather than on the Raspberry Pi.

Set:

```text
Fixed Frame: map
```

Add:

```text
Map
LaserScan
TF
RobotModel
Path
Local Costmap
Global Costmap
```

Topics:

```text
Map: /map
LaserScan: /scan_filtered
Local costmap: /local_costmap/costmap
Global costmap: /global_costmap/costmap
```

Observe:

```text
robot pose follows the real robot
scan aligns with walls
global path stays inside free space
local costmap marks nearby obstacles
map expands as frontiers are reached
previously mapped walls align when revisited
```

Do not send a manual navigation goal while `explore_lite` is controlling the
robot unless the test is intentionally changed from autonomous exploration to
manual goal navigation.

---

# 10.20 Determine Whether Exploration Is Really Complete

Do not rely only on:

```text
All frontiers traversed/tried out
```

Use all of the following:

```text
RViz map inspection
explore_debug.log
number of free and unknown cells
frontier goal success/failure history
physical observation of the environment
```

Check map cell counts using the script in Section 10.11.

A completed map should have:

```text
the required room or maze boundaries visible
no important reachable region left unknown
walls aligned without major duplication
robot trajectory covering the intended test area
```

---

# 10.21 Save the Autonomous Map

After the environment is sufficiently explored, keep SLAM Toolbox running and
stop `explore_lite` first:

```text
Ctrl+C
```

Stop Nav2 movement by ensuring no active goal remains.

Create the directory:

```bash
mkdir -p ~/maps
```

Save:

```bash
ros2 run nav2_map_server map_saver_cli \
-f ~/maps/maze_map
```

Expected:

```text
/home/kobuki/maps/maze_map.pgm
/home/kobuki/maps/maze_map.yaml
```

Verify:

```bash
ls -lh ~/maps/maze_map.*
```

Inspect:

```bash
cat ~/maps/maze_map.yaml
```

Expected fields include:

```yaml
image: maze_map.pgm
mode: trinary
resolution: 0.05
origin:
negate: 0
occupied_thresh:
free_thresh:
```

Keep the `.pgm` and `.yaml` files together.

---

# 10.22 Download the Map to the Laptop

From the laptop:

```bash
mkdir -p qbot2_maze_map
```

```bash
scp kobuki@raspberrypi.local:~/maps/maze_map.pgm \
qbot2_maze_map/
```

```bash
scp kobuki@raspberrypi.local:~/maps/maze_map.yaml \
qbot2_maze_map/
```

Verify locally:

```bash
ls -lh qbot2_maze_map
```

---

# 10.23 Clean Shutdown Order

Stop autonomous components in this order using `Ctrl+C`:

```text
1. explore_lite
2. velocity bridge
3. Nav2
4. SLAM Toolbox
5. laser filter
6. depthimage_to_laserscan
7. camera-info fixer
8. original Kinect depth node
9. Robot State Publisher
10. Kobuki driver
```

Check stopped jobs:

```bash
jobs
```

Remove any remaining stopped jobs:

```bash
kill $(jobs -p)
```

Shut down the Raspberry Pi safely:

```bash
sudo shutdown -h now
```

Wait for shutdown to complete before disconnecting power.

---

# 10.24 One-Page Startup Checklist

## Before startup

```text
[ ] RGB-D Kinect driver stopped
[ ] RTAB-Map stopped
[ ] manual teleop stopped
[ ] temporary static camera publisher stopped
[ ] no stopped Ctrl+Z jobs
[ ] configuration backups exist
[ ] robot placed safely
```

## Start

```text
[ ] Tab 1 Kobuki
[ ] Tab 2 original Kinect depth
[ ] Tab 3 camera-info fixer
[ ] Tab 4 depth-to-LaserScan
[ ] Tab 5 Robot State Publisher
[ ] Tab 6 laser filter
[ ] Tab 7 SLAM Toolbox
[ ] wait for valid /map
[ ] Tab 8 Nav2
[ ] all lifecycle nodes active
[ ] global costmap covers /map
[ ] Tab 9 velocity bridge
[ ] /commands/velocity has exactly one publisher and one subscriber
[ ] Tab 10 top, optional
[ ] Tab 11 explore_lite
```

## During exploration

```text
[ ] /odom continues publishing
[ ] /scan_filtered continues publishing
[ ] map → odom continues updating
[ ] map grows
[ ] frontier goals are accepted
[ ] Nav2 creates valid paths
[ ] robot motion is smooth
[ ] no obstacle or drop-off risk
```

## At completion

```text
[ ] verify map visually
[ ] inspect frontier debug log
[ ] stop explore_lite
[ ] save maze_map.pgm
[ ] save maze_map.yaml
[ ] copy both files to laptop
[ ] stop all nodes with Ctrl+C
[ ] shut down Pi safely
```

---

# 10.25 Full Autonomous Pass Criteria

The system is ready for full autonomous exploration only when all of the
following are true.

## Sensor and mapping

```bash
ros2 topic echo /odom --once --field header
ros2 topic echo \
/kinect/depth/image_raw \
--once \
--field header
ros2 topic echo \
/kinect/depth/camera_info_fixed \
--once \
--field header
ros2 topic echo /scan --once --field header
ros2 topic echo /scan_filtered --once --field header
ros2 topic echo /map --once --field info
```

## TF

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
odom \
base_footprint
```

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
base_footprint \
kinect_depth_frame
```

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
kinect_depth_frame \
kinect_depth_optical_frame
```

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
map \
odom
```

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
map \
base_footprint
```

## Nav2

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /smoother_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /waypoint_follower
ros2 lifecycle get /velocity_smoother
```

All required nodes must show:

```text
active [3]
```

## Costmaps

```bash
ros2 topic echo \
/local_costmap/costmap \
--once \
--field info
```

```bash
ros2 topic echo \
/global_costmap/costmap \
--once \
--field info
```

The global costmap must cover the current `/map`.

## Velocity route

```bash
ros2 topic info /commands/velocity -v
```

Required:

```text
Publisher count: 1
Subscription count: 1
```

## Exploration

The explore terminal should show:

```text
Connected to move_base nav2 server
found one or more frontiers
Sending goal
Goal ACCEPTED
```

The robot must then produce command velocities and physically move safely.

---

# Next Phase — Saved Map, Localization, and Shortest Path

After saving `maze_map.yaml` and `maze_map.pgm`:

1. Stop `explore_lite`.
2. Stop online SLAM Toolbox.
3. Stop the live mapping `map → odom` publisher.
4. Start `map_server` with `maze_map.yaml`.
5. Start AMCL.
6. Start Nav2 in localization/navigation mode.
7. Set the robot's initial pose.
8. Send a destination pose.
9. Use NavFn with `use_astar: true`.
10. record the planned path and compare path lengths if required.

Saved-map pipeline:

```text
maze_map.yaml
      │
      ▼
  map_server
      │
      ▼
     AMCL
      │
      ▼
  map → odom
      │
      ▼
     Nav2
      │
      ▼
shortest planned path
      │
      ▼
velocity_smoother
      │
      ▼
cmd_vel_bridge
      │
      ▼
Kobuki
```

Do not run online SLAM Toolbox and AMCL as competing `map → odom` publishers.

---

# Final Result

The complete 2D autonomous system uses:

```text
Kobuki odometry
original Kinect v1 depth driver
corrected camera calibration
depth-to-LaserScan conversion
filtered LaserScan data
Robot State Publisher TF
SLAM Toolbox live mapping
Nav2 global and local planning
velocity smoothing
the Kobuki velocity bridge
explore_lite frontier exploration
map saving for later localization
```

The 3D RGB-D modifications remain isolated in:

```text
/home/kobuki/kinect_rgb_ws
```

They do not damage the original 2D autonomous system as long as the two
operating modes are not launched simultaneously.
