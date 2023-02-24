cmake_minimum_required(VERSION 3.0.2)
project(path_extraction)

find_package(catkin REQUIRED
    rospy
    std_srvs
    std_msgs
    message_generation
)

#catkin_package(CATKIN_DEPENDS std_msgs message_runtime)


add_service_files(
  FILES
  StartRacing.srv
  ResetAndRestore.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
)
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/path_extraction_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
