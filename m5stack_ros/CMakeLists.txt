cmake_minimum_required(VERSION 2.8.3)
project(m5stack_ros)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

file(GLOB NODE_SCRIPTS_FILES node_scripts/*)
catkin_install_python(
  PROGRAMS ${NODE_SCRIPTS_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories()
