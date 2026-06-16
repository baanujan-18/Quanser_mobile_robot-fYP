# qbot2_bringup

## Purpose

This ROS 2 package contains launch files used to start important QBot 2 system nodes.

The package is mainly used for:

* Converting Kinect v1 depth data into laser scan data
* Launching the Robot State Publisher
* Supporting SLAM Toolbox mapping
* Supporting future Nav2 autonomous navigation

## Files in This Package

```text
qbot2_bringup/
├── package.xml
├── CMakeLists.txt
└── launch/
    ├── depth_to_scan.launch.py
    └── robot_state.launch.py
```

## Launch Files

### depth_to_scan.launch.py

This launch file starts the `depthimage_to_laserscan` node.

It converts:

```text
/kinect/depth/image_raw
```

into:

```text
/scan
```

The `/scan` topic is required by SLAM Toolbox and Nav2.

### robot_state.launch.py

This launch file starts the Robot State Publisher.

It reads the QBot 2 URDF file from the `qbot2_description` package and publishes the robot TF frames.

## Important Topics

```text
/kinect/depth/image_raw
/kinect/depth/camera_info_fixed
/scan
/tf
/tf_static
```

## Current Status

This package is completed and tested up to Phase 9. It was used successfully during SLAM Toolbox mapping.

