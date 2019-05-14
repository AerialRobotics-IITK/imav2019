cmake_minimum_required(VERSION 2.8.3)
project(imav_sim)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  mavros_msgs
  nav_msgs
  rosbag
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imav_sim
#  CATKIN_DEPENDS geometry_msgs mav_msgs mavros_msgs nav_msgs rosbag roscpp rospy sensor_msgs std_msgs tf visualization_msgs
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(gripper_control src/gripper_control.cxx )
target_link_libraries(gripper_control
${catkin_LIBRARIES} )

