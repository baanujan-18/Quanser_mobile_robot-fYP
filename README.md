# AI-Enhanced Autonomous Navigation System Using SLAM, Machine Learning, and IoT on Quanser QBot 2

## Student Details

* **Name:** Baanujan
* **Registration Number:** E/20/030
* **Supervisor:** Dr. D.H.S. Maithripala

## Project Overview

This Final Year Project focuses on developing an autonomous navigation system for the Quanser QBot 2 mobile robot using ROS 2 Humble, Kinect v1 depth sensing, SLAM Toolbox, and Nav2.

The system uses a Raspberry Pi 4 as the onboard computer and a Windows laptop with MobaXterm for SSH access and X11 forwarding. The project has currently been completed up to Phase 9, where SLAM Toolbox was successfully used to build and save a map of the environment.

## Main Project Goal

To develop a ROS 2-based autonomous navigation system for the Quanser QBot 2 mobile robot that can perform mapping, localization, and autonomous movement using sensor data.

## Main Hardware

* Raspberry Pi 4
* Quanser QBot 2
* Kobuki mobile base
* Xbox Kinect v1
* Windows laptop with MobaXterm

## Main Software

* Ubuntu 22.04 LTS
* ROS 2 Humble
* KinectV1-Ros2
* depthimage_to_laserscan
* SLAM Toolbox
* RViz2
* Nav2 map saver

## Important Working Configuration

| Item                          | Working Configuration             |
| ----------------------------- | --------------------------------- |
| Raspberry Pi username         | `kobuki`                          |
| Kinect depth topic            | `/kinect/depth/image_raw`         |
| Fixed camera info topic       | `/kinect/depth/camera_info_fixed` |
| Laser scan topic              | `/scan`                           |
| Robot odometry topic          | `/odom`                           |
| Map topic                     | `/map`                            |
| Kobuki velocity command topic | `/commands/velocity`              |

## Completed Phases

| Phase   | Status    | Description                                  |
| ------- | --------- | -------------------------------------------- |
| Phase 0 | Completed | Windows laptop and MobaXterm setup           |
| Phase 1 | Completed | Ubuntu 22.04 installed on Raspberry Pi 4     |
| Phase 2 | Completed | Ubuntu updated and essential tools installed |
| Phase 3 | Completed | ROS 2 Humble installed and tested            |
| Phase 4 | Completed | Kobuki/QBot 2 driver installed and tested    |
| Phase 5 | Completed | Kinect v1 driver installed and tested        |
| Phase 6 | Completed | Kinect depth converted to laser scan         |
| Phase 7 | Completed | QBot 2 URDF robot description created        |
| Phase 8 | Completed | Robot State Publisher configured             |
| Phase 9 | Completed | SLAM Toolbox mapping completed and map saved |

## Current Result

The robot can successfully publish odometry, Kinect depth data, laser scan data, TF frames, and SLAM map data.

A 2D map was built using SLAM Toolbox by converting Kinect depth data into a laser scan. The generated map was saved for the next stage of autonomous navigation using Nav2.

## Current Working Pipeline

```text
Kinect v1 depth data
        ↓
/kinect/depth/image_raw
        ↓
Camera info fixer
        ↓
/kinect/depth/camera_info_fixed
        ↓
depthimage_to_laserscan
        ↓
/scan
        ↓
SLAM Toolbox
        ↓
/map
```

## Map Saving

The map was saved using:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab_map
```

Expected saved files:

```text
lab_map.pgm
lab_map.yaml
```

## Next Phase

```text
Phase 10 - Autonomous Navigation with Nav2
```

The next target is to launch Nav2 using the saved map and send navigation goals in RViz2.

## Next Planned Tasks

* Install Nav2.
* Create `nav2_params.yaml`.
* Launch Nav2 with the saved map.
* Set the initial robot pose in RViz2.
* Send 2D navigation goals.
* Test whether the robot can autonomously reach selected goal points.

## Repository Structure

```text
docs/          Project documents, setup status, dependencies, and daily logs
scripts/       Helper scripts used during setup and testing
ros2_ws/src/   Custom ROS 2 packages created for QBot 2
maps/          Saved SLAM map files
media/         Screenshots and demo evidence
presentation/ Presentation files
```

## Final Expected Demonstration

The final system should demonstrate the QBot 2 robot navigating autonomously inside a mapped environment using ROS 2, Kinect-based perception, SLAM Toolbox, and Nav2.
