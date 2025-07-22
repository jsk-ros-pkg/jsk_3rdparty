cmake_minimum_required(VERSION 2.8.3)
project(downward)

find_package(catkin REQUIRED)

catkin_package()

include(ExternalProject)

if("$ENV{ROS_DISTRO}" STRGREATER "melodic") # >= noetic
  set(CXXFLAGS_NOETIC "-Wno-error=deprecated-copy")
endif()
externalproject_add(downward
  URL http://cdn.rawgit.com/jsk-ros-pkg/archives/master/Fast-Downward-f33d3b65601f.tar.gz
  TIMEOUT 120
  CONFIGURE_COMMAND ""
  BUILD_COMMAND cd src && sed -i "s@^CXXFLAGS =$@CXXFLAGS = ${CXXFLAGS_NOETIC} -Wno-deprecated-copy -Wno-maybe-uninitialized@" search/Makefile && patch -p3 < ${PROJECT_SOURCE_DIR}/fix_std_vector_namespace.patch && ./build_all DOWNWARD_BITWIDTH=native && patch -p3 < ${PROJECT_SOURCE_DIR}/fix_time_clock.patch
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
