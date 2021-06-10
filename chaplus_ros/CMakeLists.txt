cmake_minimum_required(VERSION 2.8.3)
project(chaplus_ros)

find_package(catkin REQUIRED COMPONENTS)

## System dependencies are found with CMake's conventions

catkin_python_setup()

catkin_package()

###########
## Build ##
###########

include_directories()

#############
## Install ##
#############

file(GLOB SCRIPTS_FILES scripts/*)

catkin_install_python(
  PROGRAMS ${SCRIPTS_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY sample/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

