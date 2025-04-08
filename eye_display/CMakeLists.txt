cmake_minimum_required(VERSION 3.0.2)
project(eye_display)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

include_directories()

catkin_install_python(PROGRAMS
  node_scripts/build.py node_scripts/demo_move_eye.py node_scripts/pub_eye_status.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/node_scripts)
