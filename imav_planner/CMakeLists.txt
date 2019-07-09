cmake_minimum_required(VERSION 2.8.3)
project(imav_planner)

find_package(catkin REQUIRED)
find_package(catkin_simple REQUIRED)
catkin_simple()

add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIRS})

# cs_add_executable(planner_test src/planner_test.cxx)

cs_add_executable(planner src/planner.cxx)
target_link_libraries(planner ${GeographicLib_LIBRARIES})

cs_add_executable(planner2 src/planner2.cxx)
target_link_libraries(planner2 ${GeographicLib_LIBRARIES})

cs_add_executable(router src/router.cxx)
target_link_libraries(router ${GeographicLib_LIBRARIES})

cs_install()
cs_export()
