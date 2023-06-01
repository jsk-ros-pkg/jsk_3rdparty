cmake_minimum_required(VERSION 2.8.3)
project(qcr_force_sensor_ros)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
)

file(GLOB SCRIPT_PROGRAMS node_scripts/*.py)
catkin_install_python(PROGRAMS ${SCRIPT_PROGRAMS}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/node_scripts
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
