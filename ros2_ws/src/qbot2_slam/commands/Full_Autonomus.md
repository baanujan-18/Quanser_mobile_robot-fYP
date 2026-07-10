# SLAM Toolbox Mapping Commands

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


# Phase 10 - Full Autonomous Exploration Without a Saved Map

## Goal

The goal of this phase is to make the QBot 2:

- Build the map live using SLAM Toolbox.
- Detect unexplored frontiers.
- Send frontier goals to Nav2.
- Move autonomously without keyboard teleoperation.
- Save the completed maze map for later shortest-path navigation.

A previously saved map is **not required** for this phase. However, SLAM Toolbox must remain running because Nav2 and `explore_lite` require the live `/map`.

The complete pipeline is:

```text
/odom + Kinect
       ↓
/scan
       ↓
/scan_filtered
       ↓
SLAM Toolbox
       ↓
live /map + map → odom
       ↓
Nav2
       ↓
/cmd_vel_nav
       ↓
velocity_smoother
       ↓
/cmd_vel
       ↓
cmd_vel_bridge.py
       ↓
/commands/velocity
       ↓
Kobuki
```

---

# Full Autonomous Startup Order

Use separate MobaXterm tabs and start them in this order:

```text
Tab 1  - Kobuki driver
Tab 2  - Kinect depth node
Tab 3  - Camera info fixer
Tab 4  - Depth to LaserScan
Tab 5  - Robot State Publisher
Tab 6  - Laser filter
Tab 7  - SLAM Toolbox
Tab 8  - Nav2
Tab 9  - Velocity bridge
Tab 10 - Optional CPU monitor
Tab 11 - explore_lite
```

Do not start `explore_lite` until Tabs 1-9 pass their tests.

---

# Required Configuration Backups

Before editing working configuration files:

```bash
cp ~/fyp_ws/src/qbot2_bringup/config/nav2_params.yaml \
~/fyp_ws/src/qbot2_bringup/config/nav2_params_working_backup.yaml

cp ~/fyp_ws/src/qbot2_bringup/config/explore_params.yaml \
~/fyp_ws/src/qbot2_bringup/config/explore_params_working_backup.yaml
```

Do not replace the complete Nav2 parameter file unless the replacement has already been tested on the robot. Modify the last working file in small steps.

---

# Recommended `explore_params.yaml`

Create or edit:

```bash
nano ~/fyp_ws/src/qbot2_bringup/config/explore_params.yaml
```

Use:

```yaml
/**:
  ros__parameters:
    robot_base_frame: base_footprint

    costmap_topic: map
    costmap_updates_topic: map_updates

    return_to_init: false
    visualize: false

    planner_frequency: 0.15
    progress_timeout: 90.0
    transform_tolerance: 0.5

    potential_scale: 3.0
    orientation_scale: 0.0
    gain_scale: 1.0

    min_frontier_size: 0.10
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

with open(path) as file:
    yaml.safe_load(file)

print("YAML OK:", path)
PY
```

---

# Required `nav2_params.yaml` Checks

Open the last working file:

```bash
nano ~/fyp_ws/src/qbot2_bringup/config/nav2_params.yaml
```

## Controller Server

The `FollowPath` controller must include the DWB critic list:

```yaml
controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 10.0

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

      transform_tolerance: 0.50

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

Without the `critics` list, the controller fails with:

```text
No critics defined for FollowPath
```

## Planner Server

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 1.0
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.50
      use_astar: true
      allow_unknown: true
```

## Local Costmap

Verify that the local costmap uses `/scan_filtered`:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom
      robot_base_frame: base_footprint
      rolling_window: true
      width: 3.0
      height: 3.0
      resolution: 0.05
      transform_tolerance: 0.50

      robot_radius: 0.22
      footprint_padding: 0.02

      obstacle_layer:
        scan:
          topic: /scan_filtered
          data_type: LaserScan
          clearing: true
          marking: true

      inflation_layer:
        inflation_radius: 0.35
        cost_scaling_factor: 3.0
```

If the robot passes too close to obstacles, increase the inflation radius gradually:

```text
0.35 → 0.38 → 0.40
```

If narrow maze corridors become blocked, reduce it gradually.

## Global Costmap

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map
      robot_base_frame: base_footprint
      resolution: 0.05
      track_unknown_space: true
      transform_tolerance: 0.50

      robot_radius: 0.22
      footprint_padding: 0.02

      obstacle_layer:
        scan:
          topic: /scan_filtered
          data_type: LaserScan
          clearing: true
          marking: true

      inflation_layer:
        inflation_radius: 0.35
        cost_scaling_factor: 3.0
```

## Velocity Smoother

```yaml
velocity_smoother:
  ros__parameters:
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

Validate the complete file:

```bash
python3 - <<'PY'
import yaml

path = "/home/kobuki/fyp_ws/src/qbot2_bringup/config/nav2_params.yaml"

with open(path) as file:
    config = yaml.safe_load(file)

controller = config["controller_server"]["ros__parameters"]
follow_path = controller["FollowPath"]

print("YAML OK")
print("Plugin:", follow_path.get("plugin"))
print("Critics:", follow_path.get("critics"))
print("Controller frequency:", controller.get("controller_frequency"))
PY
```

---

# Tab 8 - Nav2

Start Nav2 only after Tabs 1-7 are working.

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 launch nav2_bringup navigation_launch.py \
use_sim_time:=false \
autostart:=true \
params_file:=/home/kobuki/fyp_ws/src/qbot2_bringup/config/nav2_params.yaml
```

## Purpose

Starts the Nav2 navigation system:

- `controller_server`
- `planner_server`
- `smoother_server`
- `behavior_server`
- `bt_navigator`
- `waypoint_follower`
- `velocity_smoother`
- local costmap
- global costmap

## Test Tab 8

Run each command without pressing `Ctrl+C`:

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /smoother_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /waypoint_follower
ros2 lifecycle get /velocity_smoother
```

Every node should return:

```text
active [3]
```

Check navigation actions:

```bash
ros2 action list | grep -E "navigate_to_pose|follow_path|spin|backup|wait|drive_on_heading"
```

Expected topics include:

```text
/navigate_to_pose
/follow_path
/spin
/backup
/wait
/drive_on_heading
```

Check costmaps:

```bash
ros2 topic list | grep -E "global_costmap|local_costmap"
```

Expected topics include:

```text
/global_costmap/costmap
/local_costmap/costmap
```

## Tab 8 Pass Condition

- All lifecycle nodes are active.
- `/navigate_to_pose` exists.
- `/follow_path` exists.
- Both costmaps exist.

---

# Tab 9 - Velocity Bridge

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

The complete velocity route is:

```text
controller_server
    publishes /cmd_vel_nav
            ↓
velocity_smoother
    publishes /cmd_vel
            ↓
cmd_vel_bridge
    publishes /commands/velocity
            ↓
kobuki
```

## Test Tab 9

```bash
ros2 node list | grep cmd_vel_bridge
```

Expected:

```text
/cmd_vel_bridge
```

```bash
ros2 topic info /commands/velocity -v
```

Required:

```text
Publisher count: 1
Subscription count: 1
```

The publisher should be `cmd_vel_bridge`.

The subscriber should be `kobuki`.

---

# Tab 10 - Optional CPU Monitor

```bash
top
```

High CPU load may cause:

- delayed TF,
- stale scan data,
- controller-loop delays,
- rejected or aborted goals,
- jerky wheel motion.

Press `q` to exit.

---

# Tab 11 - Autonomous Exploration

Start only after the full readiness checklist passes.

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 run explore_lite explore --ros-args \
--params-file /home/kobuki/fyp_ws/src/qbot2_bringup/config/explore_params.yaml
```

Expected startup messages include:

```text
Waiting for costmap to become available, topic: map
Waiting to connect to move_base nav2 server
Connected to move_base nav2 server
Getting initial pose of the robot
```

The robot may initially rotate, detect a frontier, move toward it, update the map, and repeat.

RViz2 is optional.

---

# Mandatory Readiness Checklist Before Exploration

## Sensor Data

```bash
ros2 topic echo /odom --once
ros2 topic echo /kinect/depth/image_raw --once --field header
ros2 topic echo /kinect/depth/camera_info_fixed --once --field header
ros2 topic echo /scan --once --field header
ros2 topic echo /scan_filtered --once --field header
ros2 topic echo /map --once --field info
```

## TF

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint kinect_depth_frame
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_footprint
```

Stop each continuous check with `Ctrl+C`.

The TF timestamps must continue advancing. A repeated position while the robot is stationary is normal, but a timestamp that freezes for several seconds is not normal.

## Nav2

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /velocity_smoother
```

All must show:

```text
active [3]
```

## Velocity Bridge

```bash
ros2 topic info /commands/velocity -v
```

Required:

```text
Publisher count: 1
Subscription count: 1
```

Do not start `explore_lite` when any mandatory check fails.

---

# Autonomous Exploration Debug Mode

When exploration stops too early:

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 run explore_lite explore --ros-args \
--params-file /home/kobuki/fyp_ws/src/qbot2_bringup/config/explore_params.yaml \
--log-level debug 2>&1 | tee ~/explore_debug.log
```

Extract important messages:

```bash
grep -E \
"found [0-9]+ frontiers|Sending goal|Goal ACCEPTED|Goal aborted|Goal was REJECTED|black list|All frontiers|No frontiers" \
~/explore_debug.log
```

---

# Autonomous Exploration Troubleshooting

## `Goal was REJECTED by the action server`

This normally means `bt_navigator` is inactive or Nav2 is not fully active.

Check:

```bash
ros2 lifecycle get /bt_navigator
```

Required:

```text
active [3]
```

Also check all Nav2 lifecycle nodes.

---

## `All frontiers traversed/tried out, stopping`

This does not always mean the room is completely mapped.

It may mean:

- frontiers were found,
- Nav2 rejected or aborted their goals,
- `explore_lite` blacklisted them,
- no non-blacklisted frontier remained.

Check the debug log and the Nav2 terminal.

Verify:

```yaml
GridBased:
  tolerance: 0.50
  allow_unknown: true
```

Verify:

```yaml
track_unknown_space: true
```

Verify:

```yaml
progress_timeout: 90.0
min_frontier_size: 0.10
```

Restarting `explore_lite` clears its current in-memory blacklist, but the same stop will happen again if the Nav2 failure remains.

---

## `No frontiers found`

Check whether the map contains both free and unknown cells:

```bash
python3 - <<'PY'
import time
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

rclpy.init()
node = rclpy.create_node("map_data_check")
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

end_time = time.time() + 10

while not messages and time.time() < end_time:
    rclpy.spin_once(node, timeout_sec=0.5)

if not messages:
    print("ERROR: No /map message received")
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

Exploration requires:

```text
Free cells > 0
Unknown cells > 0
```

---

## TF extrapolation into the future

Example:

```text
Lookup would require extrapolation into the future
```

Check:

```bash
ros2 topic hz /odom
ros2 topic hz /scan_filtered
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_footprint
```

Recommended tolerance locations:

```yaml
FollowPath:
  transform_tolerance: 0.50
```

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      transform_tolerance: 0.50
```

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      transform_tolerance: 0.50
```

```yaml
explore_params.yaml:
  transform_tolerance: 0.5
```

Increasing tolerance does not fix a TF stream that has completely stopped. Restart the failed sensor, odometry, SLAM, or Nav2 stage.

---

## Robot does not move

Check the velocity route:

```bash
ros2 node info /controller_server
ros2 node info /velocity_smoother
ros2 node info /cmd_vel_bridge
ros2 topic info /commands/velocity -v
```

Interpretation:

```text
/cmd_vel_nav empty
→ no accepted goal, no plan, or controller problem

/cmd_vel_nav has data but /cmd_vel empty
→ velocity_smoother problem

/cmd_vel has data but /commands/velocity empty
→ bridge problem

/commands/velocity has data but robot does not move
→ Kobuki driver, motor, bumper, cliff sensor, or hardware safety problem
```

---

## Controller process exists but the ROS node is missing

If:

```bash
pgrep -af controller_server
```

shows a process, but:

```bash
ros2 node list --no-daemon --spin-time 3 | grep controller_server
```

shows nothing, the process is stuck or failed before joining the ROS graph.

Stop Nav2, then remove remaining Nav2 processes:

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

Then restart Nav2.

---

## Robot hits obstacles

Check that local and global costmaps use `/scan_filtered`.

Check:

```yaml
robot_radius: 0.22
footprint_padding: 0.02
```

Increase inflation gradually:

```text
0.35 → 0.38 → 0.40
```

Do not immediately use a very large value in a narrow maze.

### Kinect limitation

The current scan configuration has:

```text
range_min: 0.45 m
approximately 63 degrees of forward view
```

Therefore, the robot may not detect:

- obstacles closer than 45 cm,
- obstacles beside it,
- obstacles behind it,
- low objects outside the selected scan rows.

Costmap tuning cannot fully compensate for missing sensor coverage.

---

## Wheels stutter or turning is not smooth

Recommended starting values:

```yaml
controller_frequency: 10.0
max_vel_x: 0.10
max_vel_theta: 0.30
acc_lim_theta: 0.50
decel_lim_theta: -0.70
```

Velocity smoother:

```yaml
smoothing_frequency: 20.0
deadband_velocity: [0.015, 0.0, 0.06]
```

Monitor CPU load:

```bash
top
```

---

# Save the Autonomous Maze Map

After the maze is sufficiently explored:

```bash
mkdir -p ~/maps
```

```bash
ros2 run nav2_map_server map_saver_cli \
-f ~/maps/maze_map
```

Expected:

```text
maze_map.pgm
maze_map.yaml
```

Verify:

```bash
ls -lh ~/maps/maze_map.*
```

---

# Next Phase - Saved Map and Shortest Path

After saving the map:

1. Stop `explore_lite`.
2. Stop online SLAM Toolbox.
3. Start `map_server` using `maze_map.yaml`.
4. Start AMCL localization.
5. Start Nav2.
6. Set the robot's initial pose.
7. Send a target pose.
8. Use NavFn with `use_astar: true` to generate the path.
9. Record or compare the planned path length.

The saved-map pipeline is:

```text
maze_map.yaml
      ↓
map_server
      ↓
AMCL
      ↓
map → odom
      ↓
Nav2 planner
      ↓
shortest path
      ↓
Nav2 controller
      ↓
Kobuki
```

---

# Full Autonomous Pass Criteria

The system is ready for autonomous exploration only when:

```bash
ros2 topic echo /odom --once
ros2 topic echo /scan_filtered --once --field header
ros2 topic echo /map --once --field info
```

all return data, and:

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /velocity_smoother
```

all return:

```text
active [3]
```

and:

```bash
ros2 topic info /commands/velocity -v
```

shows:

```text
Publisher count: 1
Subscription count: 1
```

---

# Safety Notes for Maze Testing

- Keep the emergency stop ready.
- Start in a clear area.
- Avoid stairs and drop-offs.
- Avoid low obstacles the Kinect cannot detect.
- Do not assume the map is complete only because exploration stops.
- Inspect the debug log and saved map before starting shortest-path navigation.


# Final Result

SLAM Toolbox successfully generated a 2D map using Kinect v1 depth data converted into laser scan data.

The system successfully used:

* Kobuki odometry from `/odom`
* Kinect depth images from `/kinect/depth/image_raw`
* Corrected camera information
* Raw scan data from `/scan`
* Filtered scan data from `/scan_filtered`
* Robot TF frames
* SLAM Toolbox map generation

The map was saved as:

```text
lab_map.pgm
lab_map.yaml
```

The saved map is ready for Phase 10 autonomous navigation using Nav2.
