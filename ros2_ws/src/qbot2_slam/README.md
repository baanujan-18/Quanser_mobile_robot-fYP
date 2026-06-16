# qbot2_slam

## Purpose

This folder contains SLAM-related notes and commands for the Quanser QBot 2 autonomous navigation project.

SLAM Toolbox was used to build a 2D map of the environment using odometry and laser scan data.

## Why SLAM Toolbox Was Used

The original plan considered RTAB-Map, but SLAM Toolbox was used for the working setup because it was easier to configure with the fake laser scan generated from Kinect v1 depth data.

The Kinect depth image was converted into a `/scan` topic using `depthimage_to_laserscan`, and this `/scan` topic was used by SLAM Toolbox for mapping.

## SLAM Input Topics

SLAM Toolbox used the following important topics:

```text
/odom
/scan
/tf
/tf_static
```

## SLAM Output Topics

SLAM Toolbox published:

```text
/map
/map_metadata
```

## Full SLAM Pipeline

```text
Kobuki odometry
        ↓
/odom

Kinect v1 depth data
        ↓
depthimage_to_laserscan
        ↓
/scan

/odom + /scan + /tf
        ↓
SLAM Toolbox
        ↓
/map
```

## Files in This Folder

```text
qbot2_slam/
├── README.md
└── commands/
    └── slam_toolbox_mapping_commands.md
```

## Current Status

This SLAM setup was completed and tested successfully in Phase 9.

The robot was manually driven using teleoperation remapped to `/commands/velocity`, and the map was built in RViz2.

The map was saved as:

```text
lab_map.pgm
lab_map.yaml
```

## Next Step

The saved map will be used in Phase 10 for autonomous navigation with Nav2.

