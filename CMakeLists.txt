cmake_minimum_required(VERSION 2.8.3)
project(blinky_2dnav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslaunch
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES blinky_2dnav
#  CATKIN_DEPENDS roscpp roslaunch
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Declare a cpp executable
# add_executable(blinky_2dnav_node src/blinky_2dnav_node.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(blinky_2dnav_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

## Mark executables and/or libraries for installation
# install(TARGETS blinky_2dnav blinky_2dnav_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(DIRECTORY
#   config
#   launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

