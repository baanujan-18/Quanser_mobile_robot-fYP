# qbot2_perception

## Purpose

This folder contains perception-related helper files used for the QBot 2 autonomous navigation project.

The main purpose of this folder is to support Kinect v1 depth processing.

## Why This Folder Is Needed

The Kinect v1 depth node publishes depth image data and camera information. However, the camera information was not complete enough for the `depthimage_to_laserscan` node.

Because of that, a camera info fixer script was created.

This script subscribes to:

```text
/kinect/depth/camera_info
```

and republishes corrected camera information to:

```text
/kinect/depth/camera_info_fixed
```

The fixed camera info topic is then used by the depth-to-laserscan node.

## Files in This Folder

```text
qbot2_perception/
├── README.md
└── scripts/
    └── fix_kinect_camera_info.py
```

## Perception Pipeline

```text
Kinect v1 depth node
        ↓
/kinect/depth/image_raw
        ↓
/kinect/depth/camera_info
        ↓
fix_kinect_camera_info.py
        ↓
/kinect/depth/camera_info_fixed
        ↓
depthimage_to_laserscan
        ↓
/scan
```

## Important Topics

```text
/kinect/depth/image_raw
/kinect/depth/camera_info
/kinect/depth/camera_info_fixed
/scan
```

## Current Status

This script was tested successfully up to Phase 9. It was used during SLAM Toolbox mapping to generate a valid `/scan` topic from Kinect depth data.

## Note

This folder is not a full ROS 2 build package. It is used to store helper scripts related to perception.

