cmake_minimum_required(VERSION 2.8.3)
project(blinky_2dnav)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  roscpp
  roslaunch
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

## Declare a cpp executable
add_executable(twist_to_ackermann src/twist_to_ackermann.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(twist_to_ackermann
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS twist_to_ackermann
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
roslaunch_add_file_check(launch/amcl.launch launch/gmapping.launch launch/hector_mapping.launch launch/move_base.launch)
