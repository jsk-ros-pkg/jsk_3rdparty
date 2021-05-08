cmake_minimum_required(VERSION 2.8.3)
project(lpg_planner)

find_package(catkin REQUIRED)
catkin_package()

include(ExternalProject)
externalproject_add(lpg_planner
  URL http://eracle.ing.unibs.it/lpg/lpg1.2-linux.tar.gz
  TIMEOUT 120
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ls
  INSTALL_COMMAND bash -c "cp -rf . ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}"
  BUILD_IN_SOURCE 1
)
externalproject_get_property(lpg_planner source_dir)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
  USE_SOURCE_PERMISSIONS)
