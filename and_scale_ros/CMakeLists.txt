cmake_minimum_required(VERSION 2.8.3)
project(and_scale_ros)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS
  scripts/ekew_i_driver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
