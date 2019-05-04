# imav_sim
Simulation environment for IMAV 2019 Outdoor Challenge



## Dependencies
* [ROS](www.ros.org) : Robot Operating System  
For Installing ROS refer [ROS Installation Guide](http://wiki.ros.org/ROS/Installation). Preferably install `ROS-desktop-full`, otherwise make sure all relevant packages are installed.
* [rotors_simulator](https://github.com/ethz-asl/rotors_simulator) : RotorS is a UAV gazebo simulator.  
* [mav_control_rw](https://github.com/ethz-asl/mav_control_rw) : Control strategies for rotary wing Micro Aerial Vehicles using ROS.  


## Installation

Steps might look like : 
```shell
mkdir -p ~/imav_ws/src
cd ~/imav_ws/src
git clone https://github.com/gajena/imav_sim.git
# also clone other dependencies
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
cd ~/imav_ws
catkin build
```

## Testing

```shell
 roslaunch imav_sim three_firefly.launch
```
