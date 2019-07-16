cmake_minimum_required(VERSION 2.8.3)
project(imav_planner)

find_package(catkin REQUIRED)
find_package(catkin_simple REQUIRED)
catkin_simple()

add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

# cs_add_executable(planner_test src/planner_test.cxx)

cs_add_executable(planner src/planner.cxx)
cs_add_executable(planner2 src/planner2.cxx)
cs_add_executable(router src/router.cxx)

cs_install()
cs_export()
