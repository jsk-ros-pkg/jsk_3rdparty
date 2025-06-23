cmake_minimum_required(VERSION 2.8.3)
project(century_ros)

find_package(catkin REQUIRED)

catkin_package()

file(GLOB NODE_SCRIPTS_PY node_scripts/*.py)
catkin_install_python(PROGRAMS ${NODE_SCRIPTS_PY}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/node_scripts
)

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN "*"
  PATTERN "*/*.py" EXCLUDE
)

install(DIRECTORY launch udev
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
