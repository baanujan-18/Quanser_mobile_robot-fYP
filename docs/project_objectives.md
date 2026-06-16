# Project Objectives

## **Project Title**

**AI-Enhanced Autonomous Navigation System Using SLAM, Machine Learning, and IoT on Quanser QBot 2**

---

## **Student Details**

```text id="6p806o"
Name: Baanujan
Registration Number: E/20/030
Supervisor: Dr. D.H.S. Maithripala
Robot Platform: Quanser QBot 2
```

---

## **Main Aim**

The main aim of this project is to develop an autonomous navigation system for the Quanser QBot 2 robot using ROS 2, Kinect v1 depth sensing, SLAM-based mapping, and future Nav2-based autonomous navigation.

The system should allow the robot to understand its environment, build a map, localize itself, and navigate to a given goal position.

---

## **Main Objectives**

### **1. Set Up the QBot 2 Robot Platform**

Configure the Quanser QBot 2 robot with Raspberry Pi 4 and Ubuntu 22.04 so that it can run ROS 2 Humble and communicate with the Kobuki mobile base.

---

### **2. Establish ROS 2 Communication**

Install and configure ROS 2 Humble to manage communication between the robot base, Kinect sensor, SLAM system, and navigation system.

---

### **3. Integrate the Kobuki Mobile Base**

Set up the Kobuki driver so the QBot 2 can:

* Publish odometry data.
* Receive velocity commands.
* Move using ROS 2 topics.
* Support SLAM and navigation tasks.

Important topics:

```text id="xpc9j4"
/odom
/commands/velocity
/joint_states
```

---

### **4. Integrate Kinect v1 Depth Sensor**

Configure the Xbox Kinect v1 sensor using `KinectV1-Ros2` and `libfreenect` to publish depth image data.

Important Kinect topics:

```text id="4r2qbj"
/kinect/depth/image_raw
/kinect/depth/camera_info
```

---

### **5. Convert Depth Image to LaserScan**

Convert the Kinect v1 depth image into a 2D laser scan format using `depthimage_to_laserscan`.

This is required because SLAM Toolbox uses laser scan data for 2D mapping.

Important output topic:

```text id="79hoyv"
/scan
```

---

### **6. Create Robot Description and TF Frames**

Create a URDF model for the QBot 2 and publish the required TF frames using Robot State Publisher.

Important frames:

```text id="hopmmo"
base_footprint
base_link
camera_link
kinect_depth_frame
```

---

### **7. Build a 2D Map Using SLAM Toolbox**

Use SLAM Toolbox to build a 2D map of the environment using:

```text id="7bqyha"
/odom
/scan
/tf
/tf_static
```

SLAM Toolbox outputs:

```text id="p5qlj9"
/map
/map_metadata
```

---

### **8. Save the Generated Map**

Save the completed map for later autonomous navigation.

Saved map files:

```text id="g7x6gk"
lab_map.pgm
lab_map.yaml
```

---

### **9. Implement Autonomous Navigation Using Nav2**

Use the saved SLAM map with Nav2 to allow the QBot 2 to move autonomously to selected goal positions.

This will be completed in Phase 10.

---

### **10. Extend the System with AI and IoT Features**

After basic autonomous navigation works, the project can be extended with AI and IoT-based features such as:

* Smarter obstacle avoidance.
* Path optimization.
* Remote monitoring.
* Robot status visualization.
* Web or mobile-based control interface.
* Data logging for performance analysis.

---

## **Completed Objectives So Far**

The following objectives have been completed:

```text id="n0rqxg"
Ubuntu 22.04 setup
ROS 2 Humble setup
Kobuki/QBot 2 driver setup
Kinect v1 depth sensor setup
Depth image to LaserScan conversion
Robot URDF creation
Robot State Publisher setup
SLAM Toolbox mapping
Map saving
```

---

## **Current Project Status**

The robot can currently:

* Publish odometry.
* Publish Kinect depth data.
* Convert Kinect depth data to `/scan`.
* Publish TF frames.
* Build a 2D map using SLAM Toolbox.
* Save the generated map.

Current completed phase:

```text id="bny1n5"
Phase 9 - SLAM Toolbox Mapping
```

---

## **Next Objective**

The next objective is:

```text id="cmjz2e"
Phase 10 - Autonomous Navigation using Nav2
```

In this phase, the saved map will be loaded into Nav2 and the robot will be tested for autonomous goal-based navigation.

---

## **Expected Final Outcome**

The final expected outcome of this project is a working autonomous QBot 2 robot that can:

* Sense the environment using Kinect v1.
* Build a 2D map using SLAM.
* Save and reload the map.
* Localize itself inside the map.
* Plan a path to a target location.
* Navigate autonomously while avoiding obstacles.
* Support future AI and IoT-based enhancements.

