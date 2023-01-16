cmake_minimum_required(VERSION 2.8.3)
project(erp42_gazebo_manager)

find_package(catkin REQUIRED 
    roslaunch
    rospy
)

# add_service_files(
#   DIRECTORY srv 
#   FILES AddTwoInts.srv BadTwoInts.srv
# )

catkin_package()