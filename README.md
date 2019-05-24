# Progress Tracker

Tracking all tasks, issues and references here.
Please mention date of completion and name of task-doer (for easy follow-up) when updating this list.

## Current Tasks

### Immediate (HIGH PRIORITY)
- [ ] **PROPOSAL**
- [ ] 4I 3D Printing
- [ ] NUC repair
- [X] Budget Proposal Submission

### Testing
- [ ] MPC tuning for Flamewheel (KDE+Pixhawk)
- [X] Benchmarking vision modules with aruco
- [X] ArUco marker based landing
- [X] Deployment of detection module
- [ ] Mailbox preparation
- [ ] DroneNet video recording

### Software
- Mapping
    - [ ] Data collection
- Detection
    - [X] Load params directly from camera info file
- Multimaster
    - [X] Eliminate GUI
- Odroid
    - [ ] Downgrade ROS and OS
- House detection
    - [ ] Build model
- DroneNet
    - [ ] Transform to global coordinates

### Hardware
- [X] Odroid Setup
- [ ] Gripper Design

### Non-Technical
- [ ] Sponsor Emails
- [ ] Normal Bills
- [ ] Registration
- [ ] Team Video Template
- [ ] NUC payment feedback
- [ ] List reliable purchase sources (online and local)
- [ ] DoRD $$$ 

## To Buy/Get
- [X] Antenna-based Wifi Module
- [ ] Wifi router
- [ ] eMMC Reader
- [ ] KDE Boxes
- [ ] Arduino Nano
- [ ] Jetson carrier board
- [ ] Flamewheel 450 or other frame

## Final Adjustments
- Detection 
    - [ ] Handling illumidnation variance
    - [ ] Improve thresholding
    - [ ] Size based checks on detected objects

- Landing
    - [ ] Parameters

## Issues

### Unsolved
- [ ] RTK Delay
- [ ] Data Links
- [ ] Servo control via PixHawk
- [ ] TFMini feedback through PixHawk
- [ ] Thresholding using a lot of CPU
- [ ] **Ironman crash cause** (most probably ESC Calibration)

### Solved
(Add solved issues and a reference/short description of the solution here to help in documentation later.)
- *Going to home location when no message is published is published in offboard* : 
    Publish current odometry before going to offboard.  
- *USB Cam not working on Odroid with ROS Melodic and Ubuntu 18.04* :
    Downgrade to ROS Kinetic and Ubuntu 16.04

## References
(Maintain a list of references here that would be useful for documentation later.)
#### Misc. References (To be documented in detail later)
- [Catkin WOrkspace Config](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_config.html)
#### Planner
- [Boost MSM Documentation](https://www.boost.org/doc/libs/1_64_0/libs/msm/doc/HTML/index.html)
- [ETHZ MAV Control - State Machine Implementation](https://github.com/ethz-asl/mav_control_rw/tree/master/mav_control_interface/src)
#### DroneNet
- [YOLO](https://pjreddie.com/darknet/yolo/)
- [DroneNet on YOLO](https://github.com/chuanenlin/drone-net/)
- [DarkNet integrated with ROS](https://github.com/leggedrobotics/darknet_ros)
