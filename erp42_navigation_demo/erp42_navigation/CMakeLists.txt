cmake_minimum_required(VERSION 3.0.2)
project(erp42_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED
    rospy
    roscpp
    sensor_msgs
    cmake_modules
    std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES erp42_navigation
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
