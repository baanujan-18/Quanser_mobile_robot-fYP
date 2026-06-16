# Media

This folder contains project photos and videos related to the Quanser QBot 2 autonomous navigation project.

The media files are organized into:

* Setup evidence
* 2D mapping evidence
* 2D manual control evidence
* 2D autonomous goal navigation evidence
* Planned 3D mapping evidence

---

## **Folder Structure**

```text
media/
├── README.md
│
├── setup/
│   └── ethernet_connection/
│       ├── photos/
│       └── videos/
│
├── 2D_mapping/
│   ├── manual_control/
│   │   ├── photos/
│   │   └── videos/
│   │
│   └── autonomous_navigation/
│       ├── photos/
│       └── videos/
│
└── 3D_mapping/
    ├── manual_control/
    │   ├── photos/
    │   └── videos/
    │
    └── autonomous_navigation/
        ├── photos/
        └── videos/
```

---

## **1. Setup Evidence**

Folder:

```text
media/setup/ethernet_connection/
```

This folder contains photos and videos showing the physical communication setup.

During the project testing, the Raspberry Pi was connected to the laptop using an Ethernet cable. The laptop was used to access the Raspberry Pi remotely and run ROS 2 commands.

Recommended file name:

```text
media/setup/ethernet_connection/photos/raspberry_pi_laptop_ethernet_connection.png
```

---

## **2. 2D Mapping - Manual Control**

Folder:

```text
media/2D_mapping/manual_control/
```

This folder contains photos and videos of the QBot 2 moving under manual keyboard control.

In this test, the robot was controlled manually using teleoperation keys such as:

```text
i, j, k, m, o, u
```

The QBot 2 movement topic used was:

```text
/commands/velocity
```

Recommended video name:

```text
media/2D_mapping/manual_control/videos/qbot2_manual_control_room_demo.mp4
```

Recommended photo name:

```text
media/2D_mapping/manual_control/photos/manual_control_robot_setup.png
```

This evidence shows that the robot base movement was working correctly using manual control.

---

## **3. 2D Mapping - Autonomous Navigation**

Folder:

```text
media/2D_mapping/autonomous_navigation/
```

This folder contains photos and videos of the robot moving automatically after selecting a goal point in RViz2.

In this test, the robot was not controlled using keyboard keys. Instead, a target point was selected on the map in RViz2, and the robot moved automatically toward that point.

Recommended video name:

```text
media/2D_mapping/autonomous_navigation/videos/qbot2_rviz_goal_auto_navigation_demo.mp4
```

Recommended photo names:

```text
media/2D_mapping/autonomous_navigation/photos/rviz2_2d_map_result.png
media/2D_mapping/autonomous_navigation/photos/rviz2_goal_navigation.png
media/2D_mapping/autonomous_navigation/photos/saved_lab_map_files.png
```

This evidence shows the 2D map result and the robot movement toward a selected goal point.

---

## **4. 3D Mapping - Manual Control**

Folder:

```text
media/3D_mapping/manual_control/
```

This folder is reserved for 3D mapping work.

After 3D mapping testing, this folder can store photos and videos of the robot moving manually while collecting 3D mapping data.

Planned file names:

```text
media/3D_mapping/manual_control/videos/qbot2_3d_manual_mapping_demo.mp4
media/3D_mapping/manual_control/photos/rviz2_3d_mapping_manual_result.png
```

---

## **5. 3D Mapping - Autonomous Navigation**

Folder:

```text
media/3D_mapping/autonomous_navigation/
```

This folder is reserved for future 3D autonomous navigation or 3D visualization results.

After testing, this folder can store photos and videos of the robot moving automatically while using 3D mapping or 3D visualization.

Planned file names:

```text
media/3D_mapping/autonomous_navigation/videos/qbot2_3d_autonomous_navigation_demo.mp4
media/3D_mapping/autonomous_navigation/photos/rviz2_3d_autonomous_navigation_result.png
```

---

## **Current Media Status**

Available media:

```text
2D manual control robot movement video
2D RViz2 mapping photo/video
2D autonomous RViz2 goal navigation video
Ethernet connection setup photo
```

Planned media:

```text
3D mapping photos
3D mapping videos
3D autonomous navigation evidence
```

---

## **Upload Note**

Large videos should be compressed before uploading to GitHub.

If a video file is too large for GitHub, upload it to Google Drive or YouTube as an unlisted video and add the link in this README.

