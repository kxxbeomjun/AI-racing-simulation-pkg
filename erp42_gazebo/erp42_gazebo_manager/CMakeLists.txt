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

catkin_install_python(PROGRAMS scripts/erp42_sim_manager.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)