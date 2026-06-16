# qbot2_description

## Purpose

This ROS 2 package contains the robot description files for the Quanser QBot 2 mobile robot.

The robot description is written using URDF. It defines the basic robot body frame and the Kinect depth camera frame.

## Files in This Package

```text
qbot2_description/
├── package.xml
├── CMakeLists.txt
└── urdf/
    └── qbot2.urdf
```

## Main URDF File

```text
urdf/qbot2.urdf
```

This file defines the following robot links:

* `base_footprint`
* `base_link`
* `camera_link`
* `kinect_depth_frame`

## Important Frame

The frame `kinect_depth_frame` is important because it matches the output frame used by the depth-to-laserscan node.

## Current Status

This package is completed and tested up to Phase 9. It was used with Robot State Publisher and SLAM Toolbox mapping.

