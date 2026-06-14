# Project Objectives

## Project Title

AI-Enhanced Autonomous Navigation System Using SLAM, Machine Learning, and IoT on the Quanser Mobile Robot

## Main Objective

To develop an autonomous navigation system for the Quanser QBot 2 mobile robot using ROS 2, SLAM, sensor-based navigation, and future AI/IoT enhancements.

## Specific Objectives

1. Implement autonomous motion control and odometry using the Kobuki/QBot 2 mobile base.
2. Integrate Kinect sensor data for environment perception.
3. Use sensor data for obstacle detection and navigation support.
4. Convert Kinect depth data into laser scan data for ROS 2 navigation.
5. Build a map of the environment using SLAM.
6. Localize the robot inside the generated map.
7. Implement autonomous navigation using Nav2.
8. Add machine learning-based navigation enhancement in future stages.
9. Add IoT-based live monitoring and data logging in future stages.

## Project Scope

The project focuses on implementing an end-to-end autonomous navigation system on the available Quanser QBot 2 platform.

The initial robotics pipeline is:

```text
QBot 2 movement → sensor input → SLAM mapping → localization → autonomous navigation
