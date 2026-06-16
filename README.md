# Quanser QBot 2 Autonomous Navigation System

## **Project Title**

**AI-Enhanced Autonomous Navigation System Using SLAM, Machine Learning, and IoT on Quanser QBot 2**

---

## **Student Details**

```text
Name: Baanujan
Registration Number: E/20/030
Supervisor: Dr. D.H.S. Maithripala
Robot Platform: Quanser QBot 2
```

---

## **Project Overview**

This project focuses on developing an autonomous navigation system for the Quanser QBot 2 robot using ROS 2 Humble.

The system uses a Raspberry Pi 4 as the onboard computer, a Kobuki mobile base for movement, and an Xbox Kinect v1 depth sensor for environment sensing.

The Kinect v1 depth image is converted into a laser scan topic, which is then used by SLAM Toolbox to build a 2D map of the environment.

At the current stage, the robot can successfully build and save a 2D map using SLAM Toolbox.

---

## **Main Project Goal**

The main goal of this project is to enable the QBot 2 robot to:

* Sense the surrounding environment.
* Build a 2D map using SLAM.
* Save and reload the map.
* Localize itself inside the map.
* Navigate autonomously to a selected goal position.
* Support future AI and IoT-based extensions.

---

## **Hardware Used**

* Quanser QBot 2
* Kobuki mobile base
* Raspberry Pi 4
* Xbox Kinect v1
* Windows laptop for SSH access
* USB connection for QBot 2
* Wi-Fi connection for remote access

---

## **Software Used**

* Ubuntu 22.04 LTS
* ROS 2 Humble
* MobaXterm
* KinectV1-Ros2
* libfreenect
* depthimage_to_laserscan
* Robot State Publisher
* SLAM Toolbox
* Nav2 Map Server
* RViz2

---

## **Important Working Configuration**

| Item                    | Working Value                     |
| ----------------------- | --------------------------------- |
| Raspberry Pi username   | `kobuki`                          |
| ROS version             | `ROS 2 Humble`                    |
| Operating system        | `Ubuntu 22.04 LTS`                |
| Robot velocity topic    | `/commands/velocity`              |
| Odometry topic          | `/odom`                           |
| Kinect depth topic      | `/kinect/depth/image_raw`         |
| Fixed camera info topic | `/kinect/depth/camera_info_fixed` |
| Laser scan topic        | `/scan`                           |
| SLAM map topic          | `/map`                            |
| Kinect frame            | `kinect_depth_frame`              |

---

## **Completed Phases**

| Phase    | Description                          | Status    |
| -------- | ------------------------------------ | --------- |
| Phase 0  | Project preparation                  | Completed |
| Phase 1  | Ubuntu 22.04 setup                   | Completed |
| Phase 2  | Essential tools installation         | Completed |
| Phase 3  | ROS 2 Humble setup                   | Completed |
| Phase 4  | Kobuki/QBot 2 driver setup           | Completed |
| Phase 5  | Kinect v1 setup                      | Completed |
| Phase 6  | Kinect depth to LaserScan conversion | Completed |
| Phase 7  | Robot description using URDF         | Completed |
| Phase 8  | Robot State Publisher setup          | Completed |
| Phase 9  | SLAM Toolbox mapping                 | Completed |
| Phase 10 | Autonomous navigation using Nav2     | Pending   |

---

## **Current Working Pipeline**

```text
QBot 2 / Kobuki base
        ↓
/odom

Kinect v1 depth sensor
        ↓
/kinect/depth/image_raw

Camera info fixer
        ↓
/kinect/depth/camera_info_fixed

depthimage_to_laserscan
        ↓
/scan

Robot State Publisher
        ↓
/tf and /tf_static

SLAM Toolbox
        ↓
/map
```

---

## **Current Result**

The robot can currently:

* Launch the Kobuki/QBot 2 driver.
* Publish odometry data using `/odom`.
* Launch the Kinect v1 depth node.
* Publish Kinect depth data.
* Convert Kinect depth data into `/scan`.
* Publish robot TF frames.
* Run SLAM Toolbox.
* Build a 2D map.
* Save the generated map.

---

## **Saved Map**

The generated map was saved using:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab_map
```

Saved map files:

```text
lab_map.pgm
lab_map.yaml
```

In this repository, map files are stored in:

```text
maps/
```

---

## **Repository Structure**

```text
Quanser_mobile_robot-fYP/
│
├── README.md
├── .gitignore
│
├── docs/
│   ├── FYP_updated_setup_guide.pdf
│   ├── setup_status.md
│   ├── dependencies.md
│   ├── project_objectives.md
│   └── daily_logs/
│       ├── day_01_ros2_setup.md
│       ├── day_02_kobuki_driver.md
│       ├── day_03_kinect_v1_setup.md
│       ├── day_04_depth_to_scan.md
│       ├── day_05_robot_description.md
│       ├── day_06_robot_state_publisher.md
│       └── day_07_slam_toolbox_mapping.md
│
├── ros2_ws/
│   └── src/
│       ├── qbot2_bringup/
│       ├── qbot2_description/
│       ├── qbot2_perception/
│       ├── qbot2_slam/
│       └── qbot2_navigation/
│
├── scripts/
│   ├── phase3_install_ros2_humble.sh
│   ├── phase4_install_kobuki.sh
│   ├── phase5_install_kinect_v1.sh
│   ├── phase6_test_depth_to_scan.sh
│   └── phase9_start_slam_mapping.sh
│
├── maps/
│   ├── lab_map.pgm
│   └── lab_map.yaml
│
├── media/
│   ├── screenshots/
│   └── videos/
│
└── presentation/
    └── E20030_ES71.pptx
```

---

## **Important Note About External Packages**

The following third-party packages are used as dependencies but are not uploaded fully into this repository:

```text
kobuki_ros
kobuki_ros_interfaces
kobuki_core
velocity_smoother
ecl_tools
KinectV1-Ros2
```

They should be cloned separately during setup.

Only custom project files, documentation, scripts, maps, and results are stored in this repository.

---

## **Next Phase**

The next phase is:

```text
Phase 10 - Autonomous Navigation using Nav2
```

Planned tasks:

* Install Nav2.
* Load the saved map.
* Configure localization.
* Create `nav2_params.yaml`.
* Start Nav2.
* Set initial robot pose in RViz2.
* Send navigation goal.
* Test autonomous movement.

---

## **Final Expected Outcome**

The final expected outcome is a working QBot 2 autonomous navigation system that can:

* Build a 2D map using Kinect v1 and SLAM Toolbox.
* Save and reload the map.
* Localize itself inside the saved map.
* Plan a path to a goal.
* Navigate autonomously while avoiding obstacles.
* Support future AI and IoT-based improvements.
