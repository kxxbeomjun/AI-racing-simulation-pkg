cmake_minimum_required(VERSION 2.8.3)
project(multibot_layers)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  roscpp
)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/InflationPlugin.cfg
  cfg/OvertakePlugin.cfg
  cfg/PathPlugin.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multibot_layers
  CATKIN_DEPENDS costmap_2d dynamic_reconfigure roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(multibot_layer src/multibot_layer.cpp)
add_library(overtake_layer src/overtake_layer.cpp)
add_library(path_layer src/path_layer.cpp)

install(FILES costmap_plugins.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS multibot_layer
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(TARGETS overtake_layer
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
