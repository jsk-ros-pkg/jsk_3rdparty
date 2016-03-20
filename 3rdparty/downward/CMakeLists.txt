cmake_minimum_required(VERSION 2.8.3)
project(downward)

find_package(catkin REQUIRED)

catkin_package()

include(ExternalProject)
externalproject_add(downward
  URL http://cdn.rawgit.com/jsk-ros-pkg/archives/master/Fast-Downward-f33d3b65601f.tar.gz
  TIMEOUT 120
  CONFIGURE_COMMAND ""
  BUILD_COMMAND cd src && ./build_all DOWNWARD_BITWIDTH=native
  INSTALL_COMMAND bash -c "cp -rf --parents src/{validate,plan,preprocess/preprocess,search/downward*,search/unitcost,search/portfolio.py,translate} ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}"
  BUILD_IN_SOURCE 1
)
externalproject_get_property(downward source_dir)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
  USE_SOURCE_PERMISSIONS)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(test/test_downward.test)
endif()