# Progress Tracker

Tracking all tasks, issues and references here.
Please mention date of completion and name of task-doer (for easy follow-up) when updating this list.

## Current Tasks

### Immediate (HIGH PRIORITY)
- imav_sim repository update (GN)
- [X] **PROPOSAL**
- [X] 4I 3D Printing
- ~~[ ] NUC repair~~
- [X] Budget Proposal Submission

### Testing
- [ ] H-detector (TS+KK+AD)
    - [X] Update
    - [X] Clean
    - [X] Push
    - [X] Deploy
    - [X] Floodfill
    - [X] Slope Matching
    - [ ] Benchmark
    - [ ] High-speed datasets
- [ ] Vision testing with 3D mailbox (GN+AS)
- [ ] **Odroid benchmarking** (PM+PC)
- [X] MPC tuning for Flamewheel (KDE+Pixhawk)
- [X] Benchmarking vision modules with aruco
- [X] ArUco marker based landing
- [X] Deployment of detection module
- [X] Mailbox preparation
- [ ] DroneNet video recording

### Software
- [ ] Avoidance (GN+PM+AS) 
- [ ] Trajectory Generation (PM)
- Mapping
    - [ ] Data collection
- Detection
    - [X] Load params directly from camera info file
- Multimaster
    - [X] Eliminate GUI
- Odroid
    - [X] Downgrade ROS and OS
- House detection (AS)
    - [ ] Build model
    - [ ] Create dataset
- DroneNet
    - [ ] Transform to global coordinates

### Hardware
- [X] Odroid Setup
- [ ] Gripper Design

### Non-Technical
- [ ] Purchases (PC+PM+AP)
- [X] Finance update + Bills (PC+PM)
- [ ] Sponsor Emails
- [ ] Normal Bills
- [X] Registration
- [ ] Team Video Template
- [X] NxUC payment feedback
- [ ] List reliable purchase sources (online and local)
- [X] DoRD $$$ (Pence)

## To Buy/Get
- [X] Antenna-based Wifi Module
- [ ] Wifi router
- [X] eMMC Reader
- [X] KDE Boxes
- [X] Arduino Nano
- [X] Jetson carrier board
- [X] Flamewheel 450 or other frame
- [ ] Lidar

## Final Adjustments
- Detection 
    - [X] Handling illumination variance
    - [X] Improve thresholding
    - [X] Size based checks on detected objects

- Landing
    - [ ] Parameters

## Lectures
- MAVLink Protocol
- Transformations
- 

## Issues

### Unsolved
- [ ] RTK Delay
- [ ] Data Links
- [ ] Servo control via PixHawk
- [ ] TFMini feedback through PixHawk
- [ ] Thresholding using a lot of CPU
- [D] **Ironman crash cause** (most probably ESC Calibration)
- [X] catkin_simple : importing custom external packages fails (CMake cannot find them)

### Solved
(Add solved issues and a reference/short description of the solution here to help in documentation later.)
- *Going to home location when no message is published is published in offboard* : 
    Publish current odometry before going to offboard.  
- *USB Cam not working on Odroid with ROS Melodic and Ubuntu 18.04* :
    Downgrade to ROS Kinetic and Ubuntu 16.04
- [?] Eigen gives quad to global rotation matrix instead of global to quad(which is expected)
- HSV values change everytime : switched off auto-exposure

## References
(Maintain a list of references here that would be useful for documentation later.)
#### Misc. References (To be documented in detail later)
- [Catkin Workspace Config](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_config.html)
#### Planner
- [Boost MSM Documentation](https://www.boost.org/doc/libs/1_64_0/libs/msm/doc/HTML/index.html)
- [ETHZ MAV Control - State Machine Implementation](https://github.com/ethz-asl/mav_control_rw/tree/master/mav_control_interface/src)
#### MultiMaster
- [multimaster_fkie for IMAV](https://github.com/parekhaman1807/multimaster_fkie)
#### DroneNet
- [YOLO](https://pjreddie.com/darknet/yolo/)
- [DroneNet on YOLO](https://github.com/chuanenlin/drone-net/)
- [DarkNet integrated with ROS](https://github.com/leggedrobotics/darknet_ros)
