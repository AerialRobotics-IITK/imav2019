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
- [X] H-detector (TS+KK+AD)
    - [X] Update
    - [X] Clean
    - [X] Push
    - [X] Deploy
    - [X] Floodfill - Not Applying
    - [X] Slope Matching
    - [X] Benchmark
    - [X] High-speed datasets
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
- [ ] Gripper Design - Currently using Servos and most probably will use that.

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
- [X] Wifi router
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

## Testing Cases
- Blue Quad has delivered and since it has the size and capability to carry yellow package, should it go back to home base and pick up yellow package for delivery or just keep on Exploring?
- Travel over long distances (away from Planned Mission) to go and deliver package.
- Say red quad is detecting yellow mailbox and confirming its position and during that time yellow quad sends location of red mailbox, then the red quad should not divert from ongoing task to go and deliver the red package. Delivery should only be done from Explore state.
- Multiple mailboxes detected in same camera-frame of one quad.
- Red Quad's exploring is over but the red mailbox is not yet detected, then the red quad should not go and land, it should stay put in exploring mode and wait for red mailbox's location. Landing should only be done when whole mission is explored and package has been dropped.
- Multiple Quads detect the same mailbox.
- Battery optimization
- Quad should not abandon current task when it receives data for some other task. It should proceed for that task after completing the ongoing task.
- Yellow quad comes back to home base to pick up second package, optimize this task.
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
