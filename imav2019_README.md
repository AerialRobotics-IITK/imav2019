# imav2019

A ROS package for the International Micro Air Vehicle(IMAV) Competition, an yearly international aerial vehicle competition and conference

## Overview

This package contains the following components:

* **Planner**: A Finite State Machine implementation using the Boost C++ libraries for decision making, state transitions and actions during the mission.

* **Detector**: A detection and pose estimation module to detect the colored mailboxes in the field.

* **Helipad Detector**: A Helipad Detection module for accurate and precise landing on a helipad.

* **Router**: A message reception, checks and feedback system for keeping track of the detected mailboxes between the UAVs. Implemented with help of the [multimaster_fkie](https://github.com/fkie/multimaster_fkie) package used to sync messages among the UAVs.

* **Collision Avoidance**: A collision avoidance module for a multi-UAV system.

* **Feature Detector**: A feature detection module for detection of a house roof and a crashed UAV.

## Dependencies

* [ROS Melodic](http://wiki.ros.org/melodic) (stable, tested) with the following packages:
  
  - catkin
  
  - roscpp
  
  - OpenCV
  
  - std_msgs
  
  - sensor_msgs
  
  - nav_msgs
  
  - geometry_msgs
  
  - message_generation (for creating and using custom messages)
  
  - [catkin_simple](https://github.com/catkin/catkin_simple)
  
  - [mavros](https://github.com/mavlink/mavros)
  
  - [usb_cam](https://github.com/ros-drivers/usb_cam.git) (for obtaining images from a camera connected via USB)
  
  - [cv_bridge](https://github.com/ros-perception/vision_opencv) (OpenCV compatibility with ROS)
  
  - [cmake_modules](https://github.com/ros/cmake_modules)
  
  - [eigen_conversions](https://github.com/ros/geometry) (Eigen compatibility with ROS)
  
  - [multimaster_fkie](https://github.com/fkie/multimaster_fkie) (for Syncing messages among the UAVs)

* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)

## Installation

* Create and initialize a workspace if you have not done so already. Clone the repository and initialize the package

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin init  # initialize your catkin workspace
cd ~/catkin_ws/src
git clone git@github.com:gajena/imav2019.git
init . ./imav2019/install/install_https.rosinstall
wstool update
```

If you have a workspace, then just clone and initialize the package

```bash
cd <your-workspace>/src
git clone git@github.com:gajena/imav2019.git
init . ./imav2019/install/install_https.rosinstall
wstool update
```

Build using either `catkin build imav2019` (requires python-catkin-tools) or `catkin_make` after cloning and initializing repository


