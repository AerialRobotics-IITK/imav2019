# imav_sim
Simulation environment and software for IMAV 2019 Outdoor competition.

## Overview
This repository contains software for for IMAV 2019 Outdoor competition:-
* imav_detector : color based object detection.
* imav_optimizer : planning for optimized object delivery.
* imav_planner : high-level commander for MAV's action.
* imav_sim : contains simulation environment based on [Rotors Simluator](https://github.com/ethz-asl/rotors_simulator).
* imav_trajectory_generator : trajectory generation for mission based on [mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation).


## Installation instructions
* initialize workspace
```shell
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws
  catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
  catkin init  # initialize your catkin workspace
```

## Dependencies
* [ROS](www.ros.org) : Robot Operating System  
For Installing ROS refer [ROS Installation Guide](http://wiki.ros.org/ROS/Installation). Preferably install `kinetic`.
* [Rotors](https://github.com/ethz-asl/rotors_simulator) : RotorS is a UAV gazebo simulator. 
* [mav_control_rw](https://github.com/ethz-asl/mav_control_rw) : PID controller, and Linear, nonlinear model predictive controller with ROS.
* [mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation) : 
Polynomial trajectory generation and optimization, especially for rotary-wing MAVs.

## Published and subscribed topics
* imav_detector
  * published
    * topic_name
  * subcribed
    * topic_name

* imav_optimizer
  * published
    * topic_name
  * subcribed
    * topic_name

* imav_planner
  * published
    * topic_name
  * subcribed
    * topic_name

* imav_sim
  * published
    * topic_name
  * subcribed
    * topic_name

* imav_trajectory_generator
  * published
    * topic_name
  * subcribed
    * topic_name

## Contact
* dev_info