# qbot2_navigation

## Purpose

This folder is reserved for Nav2 autonomous navigation configuration files for the Quanser QBot 2 mobile robot.

The navigation system will use the saved SLAM map created in Phase 9 and allow the robot to move autonomously to selected goal points in RViz2.

## Current Status

This folder is prepared for Phase 10.

Phase 10 has not been fully tested yet. The final `nav2_params.yaml` file will be added after Nav2 configuration and testing are completed.

## Planned Navigation Pipeline

```text
Saved SLAM map
        ↓
Nav2 map server
        ↓
AMCL localization
        ↓
Nav2 planner
        ↓
Nav2 controller
        ↓
QBot 2 movement
```

## Expected Input Topics

```text
/map
/scan
/odom
/tf
/tf_static
```

## Expected Output

Nav2 should generate path planning and velocity commands for autonomous navigation.

The final goal is to send a `2D Nav Goal` in RViz2 and allow the QBot 2 robot to reach the selected target position autonomously.

## Files in This Folder

```text
qbot2_navigation/
├── README.md
└── config/
    └── nav2_params.yaml
```

## Note

The `nav2_params.yaml` file should be added only after Phase 10 testing is completed.

At the current stage, the saved map from Phase 9 will be used for the next navigation test.

