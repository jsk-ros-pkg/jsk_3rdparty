if("$ENV{ROS_VERSION}" EQUAL 2)
  cmake_minimum_required(VERSION 3.8)
  project(chaplus_ros)

  find_package(ament_cmake REQUIRED)

  # The installed ROS2 copies get a python3 shebang (ROS2 base images ship
  # no bare `python`), keeping the source shared with ROS1.
  find_package(jsk_ros1_ros2_compat REQUIRED)
  jsk_ros1_ros2_compat_install_python(PROGRAMS scripts/chaplus_ros.py)

  # ROS1 reads these from the source directory via rospack.
  install(FILES apikey.json communication_sample.json
    DESTINATION share/${PROJECT_NAME}
  )
  install(DIRECTORY sample/
    DESTINATION share/${PROJECT_NAME}
  )

  ament_package()

else() # if("$ENV{ROS_VERSION}" EQUAL 2)
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

endif() # if("$ENV{ROS_VERSION}" EQUAL 2)
