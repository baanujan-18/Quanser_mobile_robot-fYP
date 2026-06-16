# FYP Setup Status

## **Current Stage**

Completed up to **Phase 9 - SLAM Toolbox Mapping and Map Saving**.

## **Overall Status**

The Raspberry Pi, ROS 2 Humble, QBot 2 Kobuki base, Kinect v1 sensor, depth-to-laserscan conversion, robot description, robot state publisher, and SLAM Toolbox mapping pipeline are working successfully.

---

## **Completed Work**

## **Phase 0 - Windows Laptop Setup**

* MobaXterm installed and used for SSH connection.
* X11 forwarding enabled for RViz2 visualization.

---

## **Phase 1 - Ubuntu 22.04 Setup**

* Ubuntu 22.04 installed on Raspberry Pi 4.
* Username configured as `kobuki`.
* Raspberry Pi accessed using MobaXterm SSH.

---

## **Phase 2 - Ubuntu Update and Essential Tools**

* Ubuntu package list updated.
* System packages upgraded.
* Essential tools installed.

Commands used:

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y curl wget git build-essential python3-pip
```

---

## **Phase 3 - ROS 2 Humble Setup**

* ROS 2 Humble repository added.
* Corrected `dpkg --print-architecture` command used.
* ROS 2 Humble base installed.
* ROS 2 environment sourced.
* Talker/listener test completed successfully.

---

## **Phase 4 - Kobuki Driver Setup**

* ROS 2 workspace created at `~/fyp_ws`.
* Kobuki related packages cloned.
* Dependencies installed using `rosdep`.
* Workspace built using `colcon`.
* QBot 2 connected through USB.
* `/odom`, `/commands/velocity`, and `/joint_states` topics verified.

---

## **Phase 5 - Kinect v1 Setup**

* OpenNI2 was not used because it did not work with Kinect v1.
* `KinectV1-Ros2` package was used instead.
* `libfreenect` installed.
* Kinect hardware detected using `lsusb`.
* Kinect depth node launched successfully.
* `/kinect/depth/image_raw` topic verified.

---

## **Phase 6 - Depth to LaserScan**

* `depthimage_to_laserscan` installed.
* Camera info fixer script created.
* Kinect depth topic converted to `/scan`.
* `/scan` topic verified.

---

## **Phase 7 - Robot Description**

* QBot 2 URDF file created.
* `kinect_depth_frame` added to match the depth-to-scan output frame.
* `qbot2_description` package created and built successfully.

---

## **Phase 8 - Robot State Publisher**

* Robot State Publisher launch file created.
* TF frames published successfully.

---

## **Phase 9 - SLAM Toolbox Mapping**

* SLAM Toolbox installed.
* Full mapping pipeline launched using six MobaXterm tabs:

  * Kobuki driver
  * Kinect depth node
  * Camera info fixer
  * Depth to LaserScan
  * Robot State Publisher
  * SLAM Toolbox
* `/odom`, `/scan`, `/tf`, `/tf_static`, `/map`, and `/map_metadata` verified.
* Robot driven manually using teleop remapped to `/commands/velocity`.
* Map viewed in RViz2.
* Map saved using map saver.

---

## **Important Working Topics**

```text
/odom
/commands/velocity
/joint_states
/kinect/depth/image_raw
/kinect/depth/camera_info
/kinect/depth/camera_info_fixed
/scan
/tf
/tf_static
/map
/map_metadata
```

---

## **Current Saved Map**

The map was saved using:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab_map
```

Expected saved files:

```text
lab_map.pgm
lab_map.yaml
```

---

## **Current Result**

The robot can successfully publish odometry, Kinect depth data, laser scan data, TF frames, and SLAM map data.

A 2D map was built using SLAM Toolbox by converting Kinect v1 depth data into `/scan`. The generated map was saved and is ready for the next phase of autonomous navigation using Nav2.

---

## **Next Immediate Task**

Start **Phase 10 - Autonomous Navigation with Nav2**.

Next work:

* Install Nav2.
* Create `nav2_params.yaml`.
* Launch Nav2 with the saved map.
* Set `2D Pose Estimate` in RViz2.
* Send `2D Nav Goal`.
* Test autonomous navigation.
