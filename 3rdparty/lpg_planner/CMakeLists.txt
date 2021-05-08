cmake_minimum_required(VERSION 2.8.3)
project(lpg_planner)

find_package(catkin REQUIRED)
catkin_package()

include(ExternalProject)
externalproject_add(LPG-1.2-linux
  URL http://eracle.ing.unibs.it/lpg/lpg1.2-linux.tar.gz
  TIMEOUT 30
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND bash -c "mkdir -p ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bin; cp lpg-1.2 ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bin/"
  BUILD_IN_SOURCE 1
)
