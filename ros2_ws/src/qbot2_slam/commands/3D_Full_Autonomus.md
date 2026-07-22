# Quanser QBot 2 — Complete RGB-D 3D Mapping and Full Autonomous Exploration Guide

> **Important definition**
>
> In this project, **3D autonomous mapping** means:
>
> - RTAB-Map uses the Kinect v1 RGB and registered depth streams to build and
>   store an accumulated colored 3D map;
> - RTAB-Map also publishes a live 2D occupancy-grid projection on `/map`;
> - `explore_lite` detects frontiers on that 2D occupancy grid;
> - Nav2 plans and controls the floor-driving QBot 2 in 2D;
> - the robot moves autonomously while the RGB-D 3D database grows.
>
> Nav2 Humble does not perform native free-space planning through a full 3D
> volume for a ground robot. Its planner and controller use a 2D costmap. This
> document therefore describes the correct practical architecture:
>
> ```text
> 3D RGB-D SLAM and reconstruction
>              +
> 2D autonomous ground navigation
> ```
>
> This is the recommended architecture for the Quanser QBot 2.

---

# Project Information

```text
Student: Baanujan
Registration Number: E/20/030
Supervisor: Dr. D.H.S. Maithripala
Robot: Quanser QBot 2
Mobile base: Kobuki
Onboard computer: Raspberry Pi 4
Operating system: Ubuntu
ROS distribution: ROS 2 Humble
Depth/RGB sensor: Kinect v1
3D SLAM: RTAB-Map
Navigation: Nav2
Exploration: explore_lite
```

---

# Current Implementation Status

The following RGB-D topics have already been confirmed:

```text
/image_raw
/camera_info
/depth/image_raw
/depth/camera_info
```

The confirmed RGB stream is:

```text
height: 480
width: 640
encoding: rgb8
frame_id: kinect_depth_optical_frame
timestamp: nonzero
```

The confirmed registered depth stream is:

```text
height: 480
width: 640
encoding: 16UC1
frame_id: kinect_depth_optical_frame
timestamp: nonzero
```

The confirmed RGB and registered-depth camera matrices match.

This means the Kinect RGB-D driver is ready for the first RTAB-Map mapping
test.

> **Validation status**
>
> RGB-D publication has been tested.
>
> Full RTAB-Map + Nav2 + `explore_lite` autonomous operation has not yet been
> validated on the physical robot. Follow every pass condition in this guide
> before allowing the robot to explore without manual control.

---

# Main Goal

The goal is to make the QBot 2:

- start in an unmapped indoor environment;
- use Kinect RGB and registered depth;
- build an accumulated colored 3D RTAB-Map database;
- create a live 2D occupancy grid from RGB-D depth;
- publish the `map → odom` transform;
- detect unknown-space frontiers;
- send frontier goals to Nav2;
- plan and follow collision-aware paths;
- drive without keyboard input;
- continuously update the 3D map and 2D navigation map;
- save the RTAB-Map database;
- save the 2D occupancy map;
- later reload the RTAB-Map database for localization and goal navigation.

---

# Complete System Architecture

```text
Kinect v1 RGB stream
/image_raw
/camera_info
        │
        ├──────────────────────────────────┐
        │                                  │
        ▼                                  │
RTAB-Map RGB-D synchronization             │
        ▲                                  │
        │                                  │
Kinect registered depth                    │
/depth/image_raw                           │
/depth/camera_info                         │
        │                                  │
        ├───────────────────────┐          │
        │                       │          │
        ▼                       ▼          │
depthimage_to_laserscan     RTAB-Map SLAM ◄┘
        │                       ▲
        ▼                       │
      /scan                     │
        │                       │
        ▼                       │
 laser_filters                  │
        │                       │
        ▼                       │
 /scan_filtered                 │
        │                       │
        │              Kobuki wheel odometry
        │                    /odom
        │                       │
        │                       ▼
        │                 RTAB-Map graph SLAM
        │                       │
        │              ┌────────┴─────────┐
        │              │                  │
        │              ▼                  ▼
        │      colored 3D database       /map
        │      qbot2_rgbd_3d.db     2D occupancy grid
        │                                 │
        ├──────────────────────────┐      │
        │                          │      ▼
        ▼                          ▼  explore_lite
Nav2 local costmap       Nav2 global costmap
        │                          │
        └──────────────┬───────────┘
                       ▼
                     Nav2
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

# Operating-Mode Warning

Keep the original 2D system and the new RGB-D 3D system separate.

## Stop these nodes before using 3D mode

```text
Old ros2_kinect_depth depth_node
fix_kinect_camera_info.py
SLAM Toolbox
old 2D-only RTAB-Map tests
manual teleop_twist_keyboard
PointCloudXyzNode test nodes
temporary duplicate static_transform_publisher
old Nav2 processes
old explore_lite processes
```

The 3D autonomous mode uses:

```text
New kinect_ros2_node
RTAB-Map
depthimage_to_laserscan using the new depth topics
laser filter
Nav2
velocity bridge
explore_lite
```

## Only one Kinect driver

Do not run both:

```text
ros2_kinect_depth depth_node
```

and:

```text
kinect_ros2 kinect_ros2_node
```

at the same time.

Only one process should access the Kinect USB device.

## Only one `map → odom` publisher

Do not run SLAM Toolbox and RTAB-Map at the same time during this mode.

Both may attempt to publish:

```text
map → odom
```

A duplicate transform creates unstable localization and navigation.

## Only one motion-command path

During autonomous operation, do not run keyboard teleoperation.

The required command path is:

```text
Nav2
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

# Important Terminal Rule

Stop continuous ROS commands using:

```text
Ctrl+C
```

Do not use:

```text
Ctrl+Z
```

`Ctrl+Z` suspends the process and may leave Fast DDS shared-memory resources
active.

Check suspended jobs:

```bash
jobs
```

Remove them:

```bash
kill $(jobs -p)
```

When stale ROS nodes remain:

```bash
ros2 daemon stop
sleep 2
ros2 daemon start
```

---

# Safety Requirements

Before autonomous testing:

- use a flat indoor floor;
- keep stairs and drop-offs inaccessible;
- keep an emergency stop method ready;
- begin in a large clear area;
- keep people away from the robot;
- do not test at full speed;
- verify bumper and cliff-sensor behaviour;
- avoid transparent, reflective, very dark, or very low obstacles;
- do not assume the Kinect can see beside or behind the robot;
- supervise the robot continuously.

The Kinect-based scan has important limits:

```text
Minimum useful range: approximately 0.45 m
Horizontal field of view: approximately 63 degrees
Primary coverage: in front of the robot
```

The robot may not detect:

- obstacles closer than the configured minimum range;
- obstacles beside it;
- obstacles behind it;
- low objects outside the chosen scan rows;
- transparent surfaces;
- highly reflective surfaces;
- some dark or infrared-absorbing objects.

---

# Phase 1 — Required Software

Install the main ROS packages:

```bash
sudo apt update

sudo apt install -y \
  ros-humble-rtabmap-ros \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-map-server \
  ros-humble-depthimage-to-laserscan \
  ros-humble-laser-filters \
  ros-humble-teleop-twist-keyboard \
  ros-humble-depth-image-proc
```

Verify:

```bash
source /opt/ros/humble/setup.bash

ros2 pkg list | grep -E \
"rtabmap|nav2_bringup|depthimage_to_laserscan|laser_filters"
```

Expected RTAB-Map packages include:

```text
rtabmap_launch
rtabmap_msgs
rtabmap_slam
rtabmap_sync
rtabmap_util
```

Verify `explore_lite`:

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 pkg list | grep explore_lite
```

---

# Phase 2 — Workspace Layout

The project uses separate workspaces.

```text
~/fyp_ws
    QBot 2 bringup, Nav2, explore_lite and project configuration

~/kinect_rgb_ws
    patched Kinect v1 RGB-D ROS 2 driver

~/rtabmap_maps
    RTAB-Map database files

~/maps
    saved 2D occupancy maps
```

Create storage directories:

```bash
mkdir -p ~/rtabmap_maps
mkdir -p ~/maps
mkdir -p ~/fyp_ws/config_backups
```

---

# Phase 3 — RGB-D Driver Recovery and Build Notes

This section records the current working RGB-D driver setup.

Skip rebuilding when the following command already starts the working driver:

```bash
cd ~/kinect_rgb_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

sudo modprobe -r gspca_kinect

ros2 run kinect_ros2 kinect_ros2_node
```

## Required current RGB-D behaviour

The patched driver must:

- start the Kinect RGB video stream;
- start registered depth;
- publish valid RGB timestamps;
- publish valid depth timestamps;
- use `kinect_depth_optical_frame`;
- use matching RGB and depth intrinsics;
- use `FREENECT_DEPTH_REGISTERED`.

## Verify the source patch

```bash
grep -nE \
"FREENECT_DEPTH_REGISTERED|header.frame_id|header.stamp|CvImage" \
~/kinect_rgb_ws/src/kinect_ros2/src/kinect_ros2_component.cpp
```

Expected entries include:

```text
FREENECT_DEPTH_REGISTERED
kinect_depth_optical_frame
rgb_info_.header.stamp
CvImage(header, "rgb8", ...)
```

## Rebuild when the source changes

```bash
cd ~/kinect_rgb_ws

rm -rf \
  build/kinect_ros2 \
  install/kinect_ros2

source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-select kinect_ros2 \
  --event-handlers console_direct+
```

Expected ending:

```text
Finished <<< kinect_ros2
Summary: 1 package finished
```

---

# Phase 4 — Verify the RGB-D Driver Alone

Stop every old Kinect node first.

Start the RGB-D driver:

```bash
cd ~/kinect_rgb_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

sudo modprobe -r gspca_kinect

ros2 run kinect_ros2 kinect_ros2_node
```

Keep the terminal open.

## Topic check

```bash
source /opt/ros/humble/setup.bash
source ~/kinect_rgb_ws/install/setup.bash

ros2 topic list --no-daemon --spin-time 5 | sort | grep -E \
"image_raw|camera_info|depth|rgb|kinect"
```

Required:

```text
/camera_info
/depth/camera_info
/depth/image_raw
/image_raw
```

The old topics should not be active:

```text
/kinect/depth/image_raw
/kinect/depth/camera_info_fixed
```

## Publisher count

```bash
ros2 topic info /image_raw -v
```

```bash
ros2 topic info /depth/image_raw -v
```

Required for each:

```text
Publisher count: 1
```

## RGB rate

```bash
ros2 topic hz /image_raw
```

Expected on the current Raspberry Pi setup:

```text
approximately 8 Hz
```

Stop with `Ctrl+C`.

## Depth rate

```bash
ros2 topic hz /depth/image_raw
```

Expected on the current Raspberry Pi setup:

```text
approximately 9 to 10 Hz
```

Stop with `Ctrl+C`.

## RGB header

```bash
ros2 topic echo /image_raw --once --field header
```

Required:

```text
stamp: nonzero
frame_id: kinect_depth_optical_frame
```

## Depth header

```bash
ros2 topic echo /depth/image_raw --once --field header
```

Required:

```text
stamp: nonzero
frame_id: kinect_depth_optical_frame
```

## Camera information

```bash
ros2 topic echo /camera_info --once --field k
```

```bash
ros2 topic echo /depth/camera_info --once --field k
```

The matrices must match.

## Phase 4 pass condition

Do not continue unless:

- all four RGB-D topics exist;
- RGB and depth publish continuously;
- all timestamps are nonzero;
- RGB and depth use the same optical frame;
- RGB and registered-depth camera matrices match;
- publisher count is one for each image topic.

---

# Phase 5 — Robot TF Requirements

Start Robot State Publisher:

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 launch qbot2_bringup robot_state.launch.py
```

The required TF chain is:

```text
odom
└── base_footprint
    └── base_link
        └── kinect_depth_frame
            └── kinect_depth_optical_frame
```

## Base to camera optical frame

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
base_footprint \
kinect_depth_optical_frame
```

This must return a valid transform.

## Camera frame to optical frame

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
kinect_depth_frame \
kinect_depth_optical_frame
```

This must return a valid transform.

## Base to non-optical depth frame

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
base_footprint \
kinect_depth_frame
```

This must return a valid transform.

## Do not create duplicate camera transforms

Do not run a temporary `static_transform_publisher` when Robot State Publisher
already provides the same child frame.

Check duplicate TF publishers:

```bash
ros2 node list | grep -E \
"robot_state_publisher|static_transform"
```

## Phase 5 pass condition

Continue only when all camera transforms are available through Robot State
Publisher.

---

# Phase 6 — Kobuki Odometry and Motor Interface

Start the Kobuki driver:

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

sudo chmod 666 /dev/ttyUSB0

ros2 launch kobuki_node kobuki_node-launch.py
```

## Check odometry topic

```bash
ros2 topic echo /odom --once --field header
```

Expected:

```text
frame_id: odom
```

Check the child frame:

```bash
ros2 topic echo /odom --once | grep -E \
"frame_id|child_frame_id"
```

Expected:

```text
frame_id: odom
child_frame_id: base_footprint
```

## Check odometry rate

```bash
ros2 topic hz /odom
```

Expected:

```text
approximately 20 Hz
```

Stop with `Ctrl+C`.

## Check odometry TF

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
odom \
base_footprint
```

This must return a valid transform.

## Check the motor-command topic

```bash
ros2 topic info /commands/velocity -v
```

Required:

```text
Topic type: geometry_msgs/msg/Twist
Subscription count: at least 1
```

The subscriber should be the Kobuki driver.

---

# Phase 7 — Convert Registered Depth to LaserScan

RTAB-Map uses the full RGB-D stream for 3D SLAM.

Nav2 uses a lightweight forward LaserScan for obstacle detection.

Open a new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash
source ~/kinect_rgb_ws/install/setup.bash

ros2 run depthimage_to_laserscan \
depthimage_to_laserscan_node --ros-args \
-r depth:=/depth/image_raw \
-r depth_camera_info:=/depth/camera_info \
-r scan:=/scan \
-p scan_height:=10 \
-p range_min:=0.45 \
-p range_max:=4.0 \
-p output_frame:=kinect_depth_frame
```

## Why `kinect_depth_frame` is used

The RGB-D images use the optical frame:

```text
kinect_depth_optical_frame
```

Optical-frame axes are not the normal planar LaserScan axes.

The LaserScan output uses:

```text
kinect_depth_frame
```

Robot State Publisher must provide the transform between the two frames.

## Verify `/scan`

```bash
ros2 topic echo /scan --once --field header
```

Expected:

```text
frame_id: kinect_depth_frame
```

Check fields:

```bash
ros2 topic echo /scan --once | grep -E \
"range_min:|range_max:|angle_min:|angle_max:"
```

Expected:

```text
range_min: approximately 0.45
range_max: approximately 4.0
```

Check rate:

```bash
ros2 topic hz /scan
```

The rate will normally be close to the depth stream rate.

Stop with `Ctrl+C`.

---

# Phase 8 — Laser Filter

Use the existing laser filter:

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 run laser_filters scan_to_scan_filter_chain \
--ros-args \
--params-file /home/kobuki/fyp_ws/src/qbot2_bringup/config/laser_filter.yaml \
-r scan:=/scan \
-r scan_filtered:=/scan_filtered
```

## Verify

```bash
ros2 topic echo /scan_filtered --once --field header
```

Expected:

```text
frame_id: kinect_depth_frame
```

```bash
ros2 topic hz /scan_filtered
```

Stop with `Ctrl+C`.

Check connections:

```bash
ros2 topic info /scan -v
```

Expected:

- publisher from `depthimage_to_laserscan`;
- subscriber from the laser filter.

```bash
ros2 topic info /scan_filtered -v
```

Nav2 subscribers will appear after Nav2 starts.

---

# Phase 9 — Create the RTAB-Map Mapping Launch File

This custom launch file avoids relying on a long terminal command and keeps the
QBot 2 topic names in one place.

Create the file:

```bash
mkdir -p ~/fyp_ws/src/qbot2_bringup/launch
```

```bash
cat > ~/fyp_ws/src/qbot2_bringup/launch/qbot2_rtabmap_mapping.launch.py <<'PY'
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sync_parameters = {
        "use_sim_time": False,
        "approx_sync": True,
        "approx_sync_max_interval": 0.15,
        "topic_queue_size": 30,
        "sync_queue_size": 20,
        "qos": 1,
    }

    rtabmap_parameters = {
        "use_sim_time": False,

        # Robot frames
        "frame_id": "base_footprint",
        "odom_frame_id": "odom",
        "map_frame_id": "map",

        # TF
        "publish_tf": True,
        "wait_for_transform": 1.0,
        "tf_delay": 0.05,
        "tf_tolerance": 0.20,

        # Input mode
        "subscribe_rgbd": True,
        "subscribe_rgb": False,
        "subscribe_depth": False,
        "subscribe_scan": False,
        "subscribe_scan_cloud": False,
        "subscribe_odom_info": False,
        "approx_sync": False,

        # Queue and QoS
        "topic_queue_size": 30,
        "sync_queue_size": 20,
        "qos_image": 1,
        "qos_camera_info": 1,
        "qos_odom": 1,

        # Database
        "database_path":
            "/home/kobuki/rtabmap_maps/qbot2_rgbd_3d.db",

        # Graph SLAM
        "Mem/IncrementalMemory": "true",
        "Mem/InitWMWithAllNodes": "false",
        "Mem/NotLinkedNodesKept": "false",

        # Raspberry Pi processing rate
        "Rtabmap/DetectionRate": "1.0",

        # Ground robot constraints
        "Reg/Force3DoF": "true",
        "RGBD/ForceOdom3DoF": "true",

        # Keyframe creation
        "RGBD/LinearUpdate": "0.10",
        "RGBD/AngularUpdate": "0.17",

        # Loop closure and optimization
        "RGBD/ProximityBySpace": "true",
        "RGBD/OptimizeFromGraphEnd": "false",
        "RGBD/OptimizeMaxError": "3.0",
        "Vis/MinInliers": "12",
        "Kp/MaxDepth": "4.0",
        "Vis/MaxDepth": "4.0",

        # Create the 2D occupancy grid used by Nav2.
        # The RGB-D frames remain stored for colored 3D reconstruction.
        "RGBD/CreateOccupancyGrid": "true",
        "Grid/Sensor": "1",
        "Grid/3D": "false",
        "Grid/RayTracing": "false",
        "Grid/CellSize": "0.05",
        "Grid/DepthDecimation": "4",
        "Grid/RangeMin": "0.45",
        "Grid/RangeMax": "4.0",
        "Grid/MaxObstacleHeight": "1.50",
        "GridGlobal/UpdateError": "0.02",
        "GridGlobal/OccupancyThr": "0.55",
    }

    rgbd_sync = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync",
        namespace="rtabmap",
        output="screen",
        parameters=[sync_parameters],
        remappings=[
            ("rgb/image", "/image_raw"),
            ("depth/image", "/depth/image_raw"),
            ("rgb/camera_info", "/camera_info"),
            ("rgbd_image", "/rtabmap/rgbd_image"),
        ],
    )

    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace="rtabmap",
        output="screen",
        parameters=[rtabmap_parameters],
        remappings=[
            ("rgbd_image", "/rtabmap/rgbd_image"),
            ("odom", "/odom"),
            ("grid_map", "/map"),
        ],
    )

    return LaunchDescription([
        rgbd_sync,
        rtabmap,
    ])
PY
```

## Important database rule

This launch file does not automatically delete the database.

For a completely fresh map, manually remove the old database before launching:

```bash
rm -f ~/rtabmap_maps/qbot2_rgbd_3d.db
rm -f ~/rtabmap_maps/qbot2_rgbd_3d.db-journal
rm -f ~/rtabmap_maps/qbot2_rgbd_3d.db-wal
rm -f ~/rtabmap_maps/qbot2_rgbd_3d.db-shm
```

Do not run the delete commands when the database must be preserved.

## Build the bringup package

```bash
cd ~/fyp_ws

source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-select qbot2_bringup
```

Source:

```bash
source ~/fyp_ws/install/setup.bash
```

Verify the launch file is installed:

```bash
ros2 launch qbot2_bringup \
qbot2_rtabmap_mapping.launch.py --show-args
```

---

# Phase 10 — Why `Grid/3D` Starts as `false`

The recommended first autonomous configuration uses:

```yaml
"Grid/3D": "false"
```

This does **not** disable RGB-D 3D mapping.

RTAB-Map still stores:

- RGB images;
- registered depth images;
- calibrated camera models;
- optimized robot poses;
- visual features;
- loop closures;
- data required to reconstruct the colored 3D cloud.

The setting controls the occupancy-grid representation used online.

For the Raspberry Pi 4, the stable first target is:

```text
colored 3D RTAB-Map database
        +
live 2D occupancy grid for Nav2
```

A full 3D OctoMap consumes additional memory and CPU.

Enable it only after the base autonomous system passes.

See the optional OctoMap section near the end of this guide.

---

# Phase 11 — Test RTAB-Map Before Nav2

Keep running:

```text
Kobuki driver
Robot State Publisher
RGB-D Kinect driver
depthimage_to_laserscan
laser filter
```

Create a fresh test database:

```bash
rm -f ~/rtabmap_maps/qbot2_rgbd_3d.db*
```

Start RTAB-Map:

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash
source ~/kinect_rgb_ws/install/setup.bash

ros2 launch qbot2_bringup \
qbot2_rtabmap_mapping.launch.py
```

## Check RTAB-Map nodes

```bash
ros2 node list | grep rtabmap
```

Expected:

```text
/rtabmap/rgbd_sync
/rtabmap/rtabmap
```

## Check synchronized RGB-D

```bash
ros2 topic hz /rtabmap/rgbd_image
```

A continuous rate should appear.

It may be lower than the raw camera rates.

Stop with `Ctrl+C`.

## Check RTAB-Map subscriptions

```bash
ros2 node info /rtabmap/rtabmap
```

Required subscriptions include:

```text
/odom
/rtabmap/rgbd_image
```

## Check RTAB-Map information

```bash
ros2 topic echo /rtabmap/info --once
```

## Check the 2D occupancy map

```bash
ros2 topic echo /map --once --field info
```

Expected fields include:

```text
resolution
width
height
origin
```

The map may be very small until the robot moves.

## Check `map → odom`

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
map \
odom
```

This must return a transform.

## Check the full TF chain

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
map \
base_footprint
```

This must return a transform.

## Check the database

```bash
ls -lh ~/rtabmap_maps/qbot2_rgbd_3d.db
```

The file should exist and grow after new keyframes are added.

## Manual movement test before autonomy

Stop Nav2, bridge and `explore_lite` if any are running.

Start direct keyboard control:

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
--ros-args \
-r cmd_vel:=/commands/velocity
```

Drive slowly:

- move forward 20 to 30 cm;
- stop;
- rotate gradually;
- stop;
- move again;
- revisit the starting view.

Watch the RTAB-Map terminal for:

- new nodes;
- processing statistics;
- loop-closure messages;
- transform errors;
- synchronization warnings.

Stop teleoperation using `Ctrl+C`.

## Phase 11 pass condition

Do not continue to Nav2 unless:

- `/rtabmap/rgbd_image` publishes continuously;
- `/rtabmap/info` returns;
- `/map` returns a nonzero width and height;
- `map → odom` exists;
- `map → base_footprint` exists;
- the database grows while the robot moves;
- RTAB-Map does not repeatedly reject the RGB-D data;
- no duplicate `map → odom` publisher exists.

---

# Phase 12 — RTAB-Map Map-Content Check

Check whether the map contains free, occupied and unknown cells:

```bash
python3 - <<'PY'
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

rclpy.init()
node = rclpy.create_node("rtabmap_grid_check")
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
    print("ERROR: No /map message received")
else:
    msg = messages[-1]
    values = msg.data

    unknown = sum(value == -1 for value in values)
    free = sum(value == 0 for value in values)
    occupied = sum(value >= 65 for value in values)

    print("Width:", msg.info.width)
    print("Height:", msg.info.height)
    print("Resolution:", msg.info.resolution)
    print("Unknown:", unknown)
    print("Free:", free)
    print("Occupied:", occupied)

node.destroy_node()
rclpy.shutdown()
PY
```

For frontier exploration:

```text
Free > 0
Unknown > 0
```

Occupied cells should appear after walls or objects are observed.

If all cells remain unknown, do not start autonomous exploration.

---

# Phase 13 — Create a Separate Nav2 Configuration

Do not overwrite the final working 2D configuration.

Create a 3D/RGB-D-specific copy:

```bash
cp ~/fyp_ws/src/qbot2_bringup/config/nav2_params.yaml \
~/fyp_ws/src/qbot2_bringup/config/nav2_rtabmap_params.yaml
```

Back up the new copy:

```bash
cp ~/fyp_ws/src/qbot2_bringup/config/nav2_rtabmap_params.yaml \
~/fyp_ws/config_backups/nav2_rtabmap_params_$(date +%Y%m%d_%H%M%S).yaml
```

Open:

```bash
nano ~/fyp_ws/src/qbot2_bringup/config/nav2_rtabmap_params.yaml
```

Do not replace the complete Nav2 file with a short fragment.

Keep all required server sections.

---

# Phase 14 — Critical Nav2 Settings for RTAB-Map

The following sections must be checked in the complete
`nav2_rtabmap_params.yaml`.

## Behavior Tree Navigator

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

Retain the complete navigator-plugin configuration already present in the
working file.

## Controller Server

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

      vx_samples: 10
      vy_samples: 1
      vtheta_samples: 10

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

The critic list is mandatory.

Without it, the controller fails with:

```text
No critics defined for FollowPath
```

## Planner Server

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

`allow_unknown: true` is required for frontier goals near unknown space.

## Smoother Server

Retain the complete existing smoother configuration.

Example:

```yaml
smoother_server:
  ros__parameters:
    use_sim_time: false
    smoother_plugins: ["simple_smoother"]

    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: true
```

## Behavior Server

```yaml
behavior_server:
  ros__parameters:
    use_sim_time: false
    cycle_frequency: 5.0
    global_frame: odom
    robot_base_frame: base_footprint
    transform_tolerance: 1.0
```

Retain the complete behaviour-plugin list.

---

# Phase 15 — Local Costmap Configuration

The local costmap uses the live filtered scan.

Use integer dimensions on ROS 2 Humble:

```yaml
width: 3
height: 3
```

Do not use:

```yaml
width: 3.0
height: 3.0
```

Critical local-costmap structure:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: false

      update_frequency: 3.0
      publish_frequency: 1.0

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
          max_obstacle_height: 1.50
          clearing: true
          marking: true
          data_type: LaserScan
          raytrace_max_range: 4.0
          raytrace_min_range: 0.45
          obstacle_max_range: 3.5
          obstacle_min_range: 0.45

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.35
        cost_scaling_factor: 3.0

      always_send_full_costmap: true
```

If the robot approaches obstacles too closely, increase:

```text
0.35 → 0.38 → 0.40
```

Do not immediately use a large value in a narrow maze.

---

# Phase 16 — Global Costmap Configuration

The global costmap must use the live `/map` published by RTAB-Map.

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

      rolling_window: false

      plugins:
        - "static_layer"
        - "obstacle_layer"
        - "inflation_layer"

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
        subscribe_to_updates: false
        map_topic: /map

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true

        observation_sources: scan

        scan:
          topic: /scan_filtered
          max_obstacle_height: 1.50
          clearing: true
          marking: true
          data_type: LaserScan
          raytrace_max_range: 4.0
          raytrace_min_range: 0.45
          obstacle_max_range: 3.5
          obstacle_min_range: 0.45

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.35
        cost_scaling_factor: 3.0

      always_send_full_costmap: true
```

## Why `subscribe_to_updates` is false

RTAB-Map normally republishes the full occupancy grid when its map changes.

It may not provide the same incremental `/map_updates` stream used by SLAM
Toolbox.

The static layer should therefore subscribe to the full `/map` topic.

## Do not set fixed global width and height

The global static layer should resize from the live map.

Remove fixed global values such as:

```yaml
width: 3
height: 3
origin_x: ...
origin_y: ...
```

from the global costmap when using the live RTAB-Map map.

A stale fixed global costmap can reject valid frontier goals outside its old
bounds.

---

# Phase 17 — Velocity Smoother

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

Retain the complete input and output topic configuration in the working file.

The required result is:

```text
controller output
      ↓
velocity_smoother
      ↓
/cmd_vel
```

---

# Phase 18 — Validate the Nav2 YAML

```bash
python3 - <<'PY'
import yaml

path = (
    "/home/kobuki/fyp_ws/src/qbot2_bringup/"
    "config/nav2_rtabmap_params.yaml"
)

with open(path, "r", encoding="utf-8") as file:
    config = yaml.safe_load(file)

controller = config["controller_server"]["ros__parameters"]
planner = config["planner_server"]["ros__parameters"]
local = config["local_costmap"]["local_costmap"]["ros__parameters"]
global_map = config["global_costmap"]["global_costmap"]["ros__parameters"]

print("YAML OK:", path)
print("Controller frequency:", controller.get("controller_frequency"))
print("Critics:", controller["FollowPath"].get("critics"))
print("Planner tolerance:", planner["GridBased"].get("tolerance"))
print("Allow unknown:", planner["GridBased"].get("allow_unknown"))
print("Local width type:", type(local.get("width")).__name__)
print("Local height type:", type(local.get("height")).__name__)
print("Local scan:", local["obstacle_layer"]["scan"].get("topic"))
print("Global static map:", global_map["static_layer"].get("map_topic"))
print(
    "Global subscribe updates:",
    global_map["static_layer"].get("subscribe_to_updates"),
)
PY
```

Required:

```text
YAML OK
Critics: a nonempty list
Allow unknown: True
Local width type: int
Local height type: int
Local scan: /scan_filtered
Global static map: /map
Global subscribe updates: False
```

---

# Phase 19 — Create a Separate Exploration Configuration

Create:

```bash
cat > ~/fyp_ws/src/qbot2_bringup/config/explore_rtabmap_params.yaml <<'YAML'
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
YAML
```

## Note about `/map_updates`

RTAB-Map may publish full `/map` messages without publishing
`/map_updates`.

The exploration node's essential input is the full occupancy grid on `/map`.

Verify the behaviour on the installed `explore_lite` version before autonomous
movement.

Validate:

```bash
python3 - <<'PY'
import yaml

path = (
    "/home/kobuki/fyp_ws/src/qbot2_bringup/"
    "config/explore_rtabmap_params.yaml"
)

with open(path, "r", encoding="utf-8") as file:
    config = yaml.safe_load(file)

params = config["/**"]["ros__parameters"]

print("YAML OK:", path)
print("costmap_topic:", params.get("costmap_topic"))
print("planner_frequency:", params.get("planner_frequency"))
print("progress_timeout:", params.get("progress_timeout"))
print("transform_tolerance:", params.get("transform_tolerance"))
print("min_frontier_size:", params.get("min_frontier_size"))
PY
```

---

# Phase 20 — Velocity Bridge

The Kobuki driver receives commands on:

```text
/commands/velocity
```

Nav2 normally produces:

```text
/cmd_vel
```

The bridge is required.

## Reference bridge implementation

Create only when the existing `~/cmd_vel_bridge.py` is missing or incorrect:

```bash
cat > ~/cmd_vel_bridge.py <<'PY'
#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelBridge(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_bridge")

        self.publisher = self.create_publisher(
            Twist,
            "/commands/velocity",
            10,
        )

        self.subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.forward,
            10,
        )

        self.get_logger().info(
            "Forwarding /cmd_vel to /commands/velocity"
        )

    def forward(self, message: Twist) -> None:
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()

        for _ in range(3):
            node.publisher.publish(stop)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
PY
```

Make executable:

```bash
chmod +x ~/cmd_vel_bridge.py
```

---

# Phase 21 — Complete Daily Startup Order

Use separate terminals or MobaXterm tabs.

```text
Tab 1  — Kobuki driver
Tab 2  — Robot State Publisher
Tab 3  — Kinect RGB-D driver
Tab 4  — Registered depth to LaserScan
Tab 5  — Laser filter
Tab 6  — RTAB-Map mapping
Tab 7  — Nav2
Tab 8  — Velocity bridge
Tab 9  — Optional CPU monitor
Tab 10 — Diagnostics
Tab 11 — explore_lite
```

Do not start Tab 11 until Tabs 1 through 8 pass.

---

# Tab 1 — Kobuki Driver

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

sudo chmod 666 /dev/ttyUSB0

ros2 launch kobuki_node kobuki_node-launch.py
```

Pass condition:

```text
/odom publishes
odom → base_footprint exists
/commands/velocity has a Kobuki subscriber
```

---

# Tab 2 — Robot State Publisher

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 launch qbot2_bringup robot_state.launch.py
```

Pass condition:

```text
base_footprint → kinect_depth_frame exists
kinect_depth_frame → kinect_depth_optical_frame exists
```

---

# Tab 3 — Kinect RGB-D Driver

```bash
cd ~/kinect_rgb_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

sudo modprobe -r gspca_kinect

ros2 run kinect_ros2 kinect_ros2_node
```

Pass condition:

```text
/image_raw publishes
/camera_info publishes
/depth/image_raw publishes
/depth/camera_info publishes
```

---

# Tab 4 — Registered Depth to LaserScan

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash
source ~/kinect_rgb_ws/install/setup.bash

ros2 run depthimage_to_laserscan \
depthimage_to_laserscan_node --ros-args \
-r depth:=/depth/image_raw \
-r depth_camera_info:=/depth/camera_info \
-r scan:=/scan \
-p scan_height:=10 \
-p range_min:=0.45 \
-p range_max:=4.0 \
-p output_frame:=kinect_depth_frame
```

Pass condition:

```text
/scan publishes
frame_id is kinect_depth_frame
ranges contain valid values
```

---

# Tab 5 — Laser Filter

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 run laser_filters scan_to_scan_filter_chain \
--ros-args \
--params-file /home/kobuki/fyp_ws/src/qbot2_bringup/config/laser_filter.yaml \
-r scan:=/scan \
-r scan_filtered:=/scan_filtered
```

Pass condition:

```text
/scan_filtered publishes
```

---

# Tab 6 — RTAB-Map Mapping

For a new experiment only:

```bash
rm -f ~/rtabmap_maps/qbot2_rgbd_3d.db*
```

Start:

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash
source ~/kinect_rgb_ws/install/setup.bash

ros2 launch qbot2_bringup \
qbot2_rtabmap_mapping.launch.py
```

Pass condition:

```text
/rtabmap/rgbd_sync exists
/rtabmap/rtabmap exists
/rtabmap/rgbd_image publishes
/map publishes
map → odom exists
database file exists
```

---

# Tab 7 — Nav2

Start only after `/map` and `map → odom` exist.

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 launch nav2_bringup navigation_launch.py \
use_sim_time:=false \
autostart:=true \
params_file:=/home/kobuki/fyp_ws/src/qbot2_bringup/config/nav2_rtabmap_params.yaml
```

Pass condition:

```text
controller_server active
planner_server active
smoother_server active
behavior_server active
bt_navigator active
waypoint_follower active
velocity_smoother active
```

---

# Tab 8 — Velocity Bridge

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

python3 ~/cmd_vel_bridge.py
```

Pass condition:

```text
/cmd_vel_bridge exists
/commands/velocity has one publisher
/commands/velocity has one Kobuki subscriber
```

---

# Tab 9 — CPU Monitor

```bash
top
```

Watch:

```text
kinect_ros2
rgbd_sync
rtabmap
depthimage_to_laserscan
controller_server
planner_server
```

Press:

```text
q
```

to exit `top`.

---

# Tab 10 — Mandatory Diagnostics

## Sensors

```bash
ros2 topic echo /odom --once --field header
ros2 topic echo /image_raw --once --field header
ros2 topic echo /depth/image_raw --once --field header
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

## RTAB-Map

```bash
ros2 topic hz /rtabmap/rgbd_image
```

Stop with `Ctrl+C`.

```bash
ros2 topic echo /rtabmap/info --once
```

```bash
ls -lh ~/rtabmap_maps/qbot2_rgbd_3d.db
```

## Nav2 lifecycle

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /smoother_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /waypoint_follower
ros2 lifecycle get /velocity_smoother
```

Every node must return:

```text
active [3]
```

## Navigation actions

```bash
ros2 action list | grep -E \
"navigate_to_pose|follow_path|spin|backup|wait"
```

Required:

```text
/navigate_to_pose
/follow_path
```

## Costmaps

```bash
ros2 topic echo \
/local_costmap/costmap --once --field info
```

```bash
ros2 topic echo \
/global_costmap/costmap --once --field info
```

## Velocity route

```bash
ros2 topic info /commands/velocity -v
```

Required before exploration:

```text
Publisher count: 1
Subscription count: 1
```

The publisher should be the bridge.

The subscriber should be the Kobuki driver.

---

# Phase 22 — Global Map and Costmap Bounds Check

RTAB-Map may expand `/map` while the robot explores.

The global costmap must follow the map size.

Check RTAB-Map map dimensions:

```bash
ros2 topic echo /map --once --field info
```

Check global costmap dimensions:

```bash
ros2 topic echo \
/global_costmap/costmap --once --field info
```

Compare:

```text
resolution
width
height
origin
```

The global costmap should cover the live map.

If the map grows but the global costmap remains at an old size:

1. stop `explore_lite`;
2. stop Nav2;
3. keep RTAB-Map running;
4. restart Nav2;
5. confirm the global costmap resized;
6. restart `explore_lite`.

A frontier outside the global costmap cannot receive a valid path.

---

# Phase 23 — Test One Manual Nav2 Goal First

Do not start autonomous exploration immediately.

First verify a single nearby Nav2 goal.

The robot should begin in a clear area.

Use RViz on the laptop or send a small test goal.

Example CLI goal approximately 30 cm ahead:

```bash
ros2 action send_goal \
/navigate_to_pose \
nav2_msgs/action/NavigateToPose \
"{
  pose: {
    header: {
      frame_id: map
    },
    pose: {
      position: {
        x: 0.30,
        y: 0.00,
        z: 0.00
      },
      orientation: {
        x: 0.00,
        y: 0.00,
        z: 0.00,
        w: 1.00
      }
    }
  }
}"
```

The position is in the `map` frame.

Do not use this exact coordinate when it lies inside an obstacle or outside the
map.

## Watch command topics

```bash
ros2 topic echo /cmd_vel
```

In another terminal:

```bash
ros2 topic echo /commands/velocity
```

Interpretation:

```text
/cmd_vel has data
/commands/velocity has matching data
robot moves
→ velocity route works
```

## Manual goal pass condition

Continue only when:

- the goal is accepted;
- a valid path is generated;
- `/cmd_vel` publishes;
- `/commands/velocity` publishes;
- the robot moves smoothly;
- the robot stops at the goal;
- RTAB-Map remains active;
- `/map` continues updating;
- no TF error repeats.

---

# Tab 11 — Start Autonomous Exploration

Start only after every mandatory check passes.

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 run explore_lite explore --ros-args \
--params-file /home/kobuki/fyp_ws/src/qbot2_bringup/config/explore_rtabmap_params.yaml
```

Expected startup sequence may include:

```text
Waiting for costmap to become available
Waiting to connect to Nav2
Connected to Nav2
Getting initial pose
```

The exploration node should:

1. receive `/map`;
2. find a frontier;
3. send a Nav2 goal;
4. receive goal acceptance;
5. move the robot;
6. allow RTAB-Map to update the map;
7. repeat.

---

# Phase 24 — Pause and Resume Exploration

The ROS 2 exploration package supports stopping and resuming.

Pause:

```bash
ros2 topic pub --once \
/explore/resume \
std_msgs/msg/Bool \
"{data: false}"
```

Resume:

```bash
ros2 topic pub --once \
/explore/resume \
std_msgs/msg/Bool \
"{data: true}"
```

Pause exploration before investigating unexpected motion.

---

# Phase 25 — Exploration Debug Mode

When exploration stops early:

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash

ros2 run explore_lite explore --ros-args \
--params-file /home/kobuki/fyp_ws/src/qbot2_bringup/config/explore_rtabmap_params.yaml \
--log-level debug 2>&1 | tee ~/explore_rtabmap_debug.log
```

Extract important lines:

```bash
grep -E \
"found [0-9]+ frontiers|Sending goal|Goal ACCEPTED|Goal aborted|Goal was REJECTED|black list|All frontiers|No frontiers" \
~/explore_rtabmap_debug.log
```

---

# Phase 26 — Monitor RTAB-Map During Exploration

## Database growth

```bash
watch -n 5 \
ls -lh ~/rtabmap_maps/qbot2_rgbd_3d.db
```

Stop with `Ctrl+C`.

## RTAB-Map information

```bash
ros2 topic echo /rtabmap/info
```

Stop with `Ctrl+C`.

Look for changes in:

```text
loop_closure_id
proximity_detection_id
current robot pose
processing statistics
```

## Map rate

```bash
ros2 topic hz /map
```

The map normally publishes much more slowly than the raw camera topics.

## TF freshness

```bash
ros2 run tf2_ros tf2_echo \
map \
base_footprint
```

The timestamp must continue advancing.

---

# Phase 27 — Viewing the 3D Map

Do not run `rtabmap_viz` on the Raspberry Pi during the first autonomous test.

## Recommended method

Use a more powerful laptop on the same ROS network.

In RViz2:

1. set **Fixed Frame** to `map`;
2. add **Map** using `/map`;
3. add **LaserScan** using `/scan_filtered`;
4. add **TF**;
5. add **RobotModel**;
6. search the available RTAB-Map point-cloud topics.

List possible 3D topics:

```bash
ros2 topic list | grep -Ei \
"cloud_map|octomap|mapData|mapGraph|rtabmap"
```

If a live cloud topic is not available or is too heavy, use the saved database
after the run.

## Database viewing

Copy the database to the laptop:

```bash
scp \
kobuki@raspberrypi.local:~/rtabmap_maps/qbot2_rgbd_3d.db \
.
```

Open it with RTAB-Map Database Viewer on a computer with RTAB-Map installed.

Check the executable:

```bash
which rtabmap-databaseViewer
```

Open:

```bash
rtabmap-databaseViewer \
qbot2_rgbd_3d.db
```

The database viewer can inspect the graph, images, depth frames, constraints and
3D reconstruction.

---

# Phase 28 — Save the Maps Correctly

## Stop order

Stop autonomous exploration first:

```text
Ctrl+C in the explore_lite terminal
```

Wait until Nav2 stops commanding motion.

Stop Nav2:

```text
Ctrl+C in the Nav2 terminal
```

Stop the bridge:

```text
Ctrl+C in the bridge terminal
```

Stop RTAB-Map last:

```text
Ctrl+C in the RTAB-Map terminal
```

Wait for the RTAB-Map terminal to return to the shell prompt.

This allows the database to close cleanly.

## Verify the 3D database

```bash
ls -lh \
~/rtabmap_maps/qbot2_rgbd_3d.db
```

The primary 3D mapping result is:

```text
/home/kobuki/rtabmap_maps/qbot2_rgbd_3d.db
```

## Save the 2D occupancy map

RTAB-Map must still be publishing `/map` when using `map_saver_cli`.

Therefore, save the 2D map before stopping RTAB-Map:

```bash
mkdir -p ~/maps

ros2 run nav2_map_server map_saver_cli \
-f ~/maps/qbot2_rtabmap_map
```

Expected:

```text
qbot2_rtabmap_map.pgm
qbot2_rtabmap_map.yaml
```

Verify:

```bash
ls -lh ~/maps/qbot2_rtabmap_map.*
```

## Recommended final stop sequence

Use this order:

```text
1. Stop explore_lite
2. Stop robot motion and verify zero velocity
3. Save the 2D map while RTAB-Map is running
4. Stop Nav2
5. Stop the velocity bridge
6. Stop RTAB-Map and wait for database closure
7. Stop depth-to-LaserScan
8. Stop laser filter
9. Stop the Kinect driver
10. Stop Robot State Publisher
11. Stop the Kobuki driver
```

---

# Phase 29 — Verify Zero Velocity Before Handling the Robot

After stopping exploration:

```bash
ros2 topic echo /commands/velocity --once
```

The expected stop message contains zeros:

```text
linear:
  x: 0.0
angular:
  z: 0.0
```

When necessary, publish a one-time stop:

```bash
ros2 topic pub --once \
/commands/velocity \
geometry_msgs/msg/Twist \
"{
  linear: {
    x: 0.0,
    y: 0.0,
    z: 0.0
  },
  angular: {
    x: 0.0,
    y: 0.0,
    z: 0.0
  }
}"
```

---

# Phase 30 — Create RTAB-Map Localization Launch File

After a good map is saved, RTAB-Map can reload the database in localization
mode.

Create:

```bash
cat > ~/fyp_ws/src/qbot2_bringup/launch/qbot2_rtabmap_localization.launch.py <<'PY'
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sync_parameters = {
        "use_sim_time": False,
        "approx_sync": True,
        "approx_sync_max_interval": 0.15,
        "topic_queue_size": 30,
        "sync_queue_size": 20,
        "qos": 1,
    }

    rtabmap_parameters = {
        "use_sim_time": False,

        "frame_id": "base_footprint",
        "odom_frame_id": "odom",
        "map_frame_id": "map",

        "publish_tf": True,
        "wait_for_transform": 1.0,
        "tf_delay": 0.05,
        "tf_tolerance": 0.20,

        "subscribe_rgbd": True,
        "subscribe_rgb": False,
        "subscribe_depth": False,
        "subscribe_scan": False,
        "subscribe_scan_cloud": False,
        "subscribe_odom_info": False,
        "approx_sync": False,

        "topic_queue_size": 30,
        "sync_queue_size": 20,
        "qos_image": 1,
        "qos_camera_info": 1,
        "qos_odom": 1,

        "database_path":
            "/home/kobuki/rtabmap_maps/qbot2_rgbd_3d.db",

        # Localization mode
        "Mem/IncrementalMemory": "false",
        "Mem/InitWMWithAllNodes": "true",

        "Rtabmap/DetectionRate": "1.0",

        "Reg/Force3DoF": "true",
        "RGBD/ForceOdom3DoF": "true",

        "RGBD/ProximityBySpace": "true",
        "RGBD/OptimizeFromGraphEnd": "false",
        "RGBD/OptimizeMaxError": "3.0",

        "Vis/MinInliers": "12",
        "Kp/MaxDepth": "4.0",
        "Vis/MaxDepth": "4.0",

        "RGBD/CreateOccupancyGrid": "true",
        "Grid/Sensor": "1",
        "Grid/3D": "false",
        "Grid/RayTracing": "false",
        "Grid/CellSize": "0.05",
        "Grid/DepthDecimation": "4",
        "Grid/RangeMin": "0.45",
        "Grid/RangeMax": "4.0",
        "Grid/MaxObstacleHeight": "1.50",
    }

    rgbd_sync = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync",
        namespace="rtabmap",
        output="screen",
        parameters=[sync_parameters],
        remappings=[
            ("rgb/image", "/image_raw"),
            ("depth/image", "/depth/image_raw"),
            ("rgb/camera_info", "/camera_info"),
            ("rgbd_image", "/rtabmap/rgbd_image"),
        ],
    )

    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace="rtabmap",
        output="screen",
        parameters=[rtabmap_parameters],
        remappings=[
            ("rgbd_image", "/rtabmap/rgbd_image"),
            ("odom", "/odom"),
            ("grid_map", "/map"),
        ],
    )

    return LaunchDescription([
        rgbd_sync,
        rtabmap,
    ])
PY
```

Rebuild:

```bash
cd ~/fyp_ws

source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-select qbot2_bringup
```

---

# Phase 31 — Saved-Database Localization and Goal Navigation

Start:

```text
Kobuki
Robot State Publisher
Kinect RGB-D driver
depth-to-LaserScan
laser filter
RTAB-Map localization
Nav2
velocity bridge
```

Do not start `explore_lite` for ordinary saved-map goal navigation.

## Start RTAB-Map localization

```bash
source /opt/ros/humble/setup.bash
source ~/fyp_ws/install/setup.bash
source ~/kinect_rgb_ws/install/setup.bash

ros2 launch qbot2_bringup \
qbot2_rtabmap_localization.launch.py
```

## Localization requirement

Start the robot where the Kinect can see features already stored in the
database.

Keep the robot still for several seconds.

Check:

```bash
ros2 topic echo /rtabmap/info
```

A successful localization can be indicated by nonzero values such as:

```text
loop_closure_id
proximity_detection_id
landmark_id
```

Stop the echo using `Ctrl+C`.

Check:

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
map \
odom
```

Do not send a navigation goal until `map → odom` is valid and stable.

## Start Nav2

```bash
ros2 launch nav2_bringup navigation_launch.py \
use_sim_time:=false \
autostart:=true \
params_file:=/home/kobuki/fyp_ws/src/qbot2_bringup/config/nav2_rtabmap_params.yaml
```

Then start the bridge.

Send a goal using RViz or the `/navigate_to_pose` action.

---

# Phase 32 — Optional 3D OctoMap Mode

Enable this only after the recommended base system is stable.

In both RTAB-Map launch files, change:

```python
"Grid/3D": "false",
```

to:

```python
"Grid/3D": "true",
```

Initially keep:

```python
"Grid/RayTracing": "false",
```

A 3D occupancy structure requires more memory and processing time.

Rebuild:

```bash
cd ~/fyp_ws

source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-select qbot2_bringup
```

Check possible OctoMap topics:

```bash
ros2 topic list | grep -Ei \
"octomap|cloud_map"
```

Do not enable expensive 3D visualization on the Raspberry Pi during autonomous
testing.

## Important

Even with a 3D OctoMap, Nav2 Humble still performs planning and control on its
2D costmap.

The 3D occupancy model is useful for mapping and environmental understanding,
not native aerial-style 3D trajectory planning.

---

# Phase 33 — Optional Nav2 Voxel Layer

Nav2's voxel layer can consume 3D observations and maintain a vertical voxel
model, but it projects that model down to the 2D planning costmap.

This is an advanced upgrade.

It is not recommended for the first Raspberry Pi 4 autonomous run because it
adds:

- point-cloud generation;
- 3D raycasting;
- extra memory use;
- extra CPU use;
- more TF and synchronization dependencies.

First pass the complete LaserScan-based autonomous system.

Only then test a point-cloud voxel layer in a separate Nav2 configuration.

---

# Phase 34 — Troubleshooting

## RTAB-Map says it did not receive data

Check:

```bash
ros2 topic hz /image_raw
ros2 topic hz /depth/image_raw
ros2 topic hz /rtabmap/rgbd_image
ros2 topic hz /odom
```

Check headers:

```bash
ros2 topic echo /image_raw --once --field header
ros2 topic echo /depth/image_raw --once --field header
```

Required:

```text
nonzero timestamps
same optical frame
```

Check subscriptions:

```bash
ros2 node info /rtabmap/rgbd_sync
ros2 node info /rtabmap/rtabmap
```

---

## `/rtabmap/rgbd_image` does not publish

Increase the approximate-sync interval in the launch files:

```python
"approx_sync_max_interval": 0.20,
```

Rebuild and retest.

Do not make the interval very large because unrelated RGB and depth frames may
be paired.

---

## `/map` does not appear

Check:

```bash
ros2 node list | grep rtabmap
ros2 topic echo /rtabmap/info --once
ros2 topic hz /rtabmap/rgbd_image
ros2 topic hz /odom
```

Check RTAB-Map parameter values:

```bash
ros2 param get \
/rtabmap/rtabmap \
RGBD/CreateOccupancyGrid
```

```bash
ros2 param get \
/rtabmap/rtabmap \
Grid/Sensor
```

Required:

```text
RGBD/CreateOccupancyGrid: true
Grid/Sensor: 1
```

Move the robot slowly to create additional keyframes.

---

## `map → odom` is missing

Check:

```bash
ros2 topic hz /odom
```

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
odom \
base_footprint
```

```bash
ros2 topic echo /rtabmap/info --once
```

Ensure SLAM Toolbox is not running.

Check duplicate map-frame publishers:

```bash
ros2 node list | grep -E \
"slam_toolbox|rtabmap"
```

Only RTAB-Map should own `map → odom` in this mode.

---

## RTAB-Map rejects RGB-D conversion

Check:

```bash
timeout 10 ros2 run tf2_ros tf2_echo \
base_footprint \
kinect_depth_optical_frame
```

Check all four RGB-D headers.

Verify that RGB and depth use the same dimensions:

```bash
ros2 topic echo /image_raw --once | grep -E \
"height:|width:|encoding:"
```

```bash
ros2 topic echo /depth/image_raw --once | grep -E \
"height:|width:|encoding:"
```

Verify that the camera matrices match.

---

## Database size stays at zero

Check that the robot moves enough to satisfy:

```text
RGBD/LinearUpdate: 0.10 m
RGBD/AngularUpdate: 0.17 rad
```

Check `/odom`.

Check the RTAB-Map terminal for rejected updates.

Check disk space:

```bash
df -h
```

Check database permissions:

```bash
ls -ld ~/rtabmap_maps
```

---

## Map is noisy or contains false obstacles

Reduce maximum range:

```python
"Grid/RangeMax": "3.5",
```

Increase depth decimation:

```python
"Grid/DepthDecimation": "6",
```

Increase minimum range slightly:

```python
"Grid/RangeMin": "0.50",
```

Make one change at a time.

Record the result before changing another parameter.

---

## The 3D map bends, tilts or floats

Verify:

```bash
ros2 run tf2_ros tf2_echo \
base_footprint \
kinect_depth_optical_frame
```

Check the camera mounting translation and optical rotation.

Confirm:

```python
"Reg/Force3DoF": "true"
```

Check wheel odometry for jumps.

Move more slowly.

Avoid lifting or tilting the Kinect while mapping.

---

## Loop closures do not occur

Revisit previously viewed locations slowly.

Keep textured objects and corners visible.

Avoid pointing only at blank white walls.

Check the RGB image quality.

Lower `Vis/MinInliers` only gradually:

```text
12 → 10
```

Do not lower it aggressively because false loop closures can deform the map.

---

## The map jumps after a loop closure

A small correction is normal.

Large deformation may indicate:

- incorrect camera TF;
- bad wheel odometry;
- false visual loop closure;
- weak image texture;
- excessive motion blur;
- incorrect RGB-depth registration.

Stop the robot and inspect the RTAB-Map log.

Do not continue autonomous exploration with a severely deformed map.

---

## Nav2 starts but the global costmap is empty

Check:

```bash
ros2 topic echo /map --once --field info
```

Check the static-layer configuration:

```yaml
map_topic: /map
map_subscribe_transient_local: true
subscribe_to_updates: false
```

Restart Nav2 after `/map` is already available.

---

## Frontier goal is outside global costmap

Compare map and costmap dimensions.

Restart Nav2 so the static layer reloads the current full map.

Remove fixed global width and height.

Confirm:

```yaml
rolling_window: false
```

in the global costmap.

---

## Goal is rejected

Check:

```bash
ros2 lifecycle get /bt_navigator
```

Required:

```text
active [3]
```

Check all Nav2 lifecycle nodes.

Check the `/navigate_to_pose` action.

---

## Goal is accepted but no path is created

Check the planner log.

Verify:

```yaml
allow_unknown: true
tolerance: 1.0
track_unknown_space: true
```

Verify the goal lies inside the global costmap.

Check whether the start pose is occupied.

---

## Robot does not move

Check the command route:

```bash
ros2 topic echo /cmd_vel
```

```bash
ros2 topic echo /commands/velocity
```

Interpretation:

```text
/cmd_vel empty
→ no accepted goal, planner failure or controller failure

/cmd_vel has data but /commands/velocity empty
→ bridge failure

/commands/velocity has data but robot does not move
→ Kobuki driver, safety sensor, motor or hardware issue
```

---

## Multiple velocity publishers

```bash
ros2 topic info /commands/velocity -v
```

During autonomous operation, required:

```text
Publisher count: 1
```

Stop:

```text
teleop_twist_keyboard
duplicate bridge
manual test publisher
old Nav2 stack
```

---

## Robot hits obstacles

Stop immediately.

Verify `/scan_filtered`.

Check costmap subscriptions.

Increase inflation gradually.

Remember that the Kinect does not provide 360-degree coverage.

A forward-only Kinect cannot guarantee safe autonomous navigation in every
environment.

---

## Wheels stutter

Reduce load:

- keep `rtabmap_viz` off;
- keep RViz off the Pi;
- keep `Rtabmap/DetectionRate` at 1 Hz;
- use `Grid/DepthDecimation: 4` or higher;
- keep Nav2 controller at 5 Hz;
- disable optional OctoMap;
- stop unused ROS nodes.

Check:

```bash
top
```

---

## Fast DDS shared-memory errors

Check:

```bash
jobs
```

Remove stopped jobs:

```bash
kill $(jobs -p)
```

Reset ROS discovery:

```bash
ros2 daemon stop
sleep 2
ros2 daemon start
```

Open fresh terminals and source the workspaces again.

Always stop with `Ctrl+C`.

---

# Phase 35 — Full Autonomous Readiness Checklist

Do not start `explore_lite` until every item passes.

## RGB-D

```bash
ros2 topic echo /image_raw --once --field header
ros2 topic echo /depth/image_raw --once --field header
ros2 topic echo /camera_info --once --field header
ros2 topic echo /depth/camera_info --once --field header
```

Required:

```text
nonzero timestamps
frame_id: kinect_depth_optical_frame
```

## Odometry

```bash
ros2 topic echo /odom --once
ros2 topic hz /odom
```

Required:

```text
continuous odometry
approximately 20 Hz
```

## LaserScan

```bash
ros2 topic echo /scan_filtered --once --field header
```

Required:

```text
frame_id: kinect_depth_frame
```

## RTAB-Map

```bash
ros2 topic hz /rtabmap/rgbd_image
ros2 topic echo /rtabmap/info --once
ros2 topic echo /map --once --field info
```

Required:

```text
continuous synchronized RGB-D
valid RTAB-Map information
nonzero map dimensions
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

All must work.

## Nav2

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /smoother_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /velocity_smoother
```

All must show:

```text
active [3]
```

## Velocity route

```bash
ros2 topic info /commands/velocity -v
```

Required:

```text
Publisher count: 1
Subscription count: 1
```

## Single-goal test

A nearby manually selected Nav2 goal must:

- be accepted;
- generate a path;
- move the robot;
- stop successfully;
- avoid nearby obstacles;
- keep RTAB-Map and `/map` active.

---

# Phase 36 — Full Autonomous Pass Criteria

The 3D autonomous system passes when all of the following are observed:

- the RGB-D Kinect driver publishes valid synchronized data;
- RTAB-Map continuously receives RGB-D and wheel odometry;
- the RTAB-Map database grows;
- the database contains RGB-D frames and graph nodes;
- `/map` expands as the robot explores;
- `map → odom` remains valid;
- `/scan_filtered` supplies local obstacle observations;
- all required Nav2 lifecycle nodes remain active;
- `explore_lite` repeatedly finds frontiers;
- frontier goals are accepted;
- the planner creates valid paths;
- the controller produces smooth velocity commands;
- the bridge forwards commands to the Kobuki base;
- the robot moves autonomously;
- the robot stops safely on failures;
- the final RTAB-Map database closes cleanly;
- the final 2D occupancy map is saved.

---

# Final Outputs

## Colored RGB-D 3D mapping database

```text
/home/kobuki/rtabmap_maps/qbot2_rgbd_3d.db
```

## Saved 2D navigation map

```text
/home/kobuki/maps/qbot2_rtabmap_map.pgm
/home/kobuki/maps/qbot2_rtabmap_map.yaml
```

## Exploration debug log

```text
/home/kobuki/explore_rtabmap_debug.log
```

---

# Recommended GitHub Location

Save this guide as:

```text
ros2_ws/src/qbot2_slam/commands/3D_Full_Autonomous.md
```

Recommended commit message:

```text
Add complete RGB-D 3D autonomous exploration guide
```

---

# Final Result Statement

After physical testing and successful completion of every pass condition, the
Quanser QBot 2 will use the Kinect v1 RGB-D stream and Kobuki wheel odometry
with RTAB-Map to build an accumulated colored 3D map while publishing a live
2D occupancy grid for Nav2.

`explore_lite` will detect frontiers on the 2D grid, Nav2 will plan and control
the ground robot, and the velocity bridge will forward commands to the Kobuki
base.

The final system will provide:

```text
RGB-D colored 3D mapping
RTAB-Map graph SLAM
visual loop closure
live 2D navigation map
autonomous frontier exploration
Nav2 path planning
Kobuki motion control
saved 3D database
saved 2D occupancy map
saved-map RGB-D localization
```
