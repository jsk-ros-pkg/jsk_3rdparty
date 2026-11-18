cmake_minimum_required(VERSION 2.8.3)
project(downward)

find_package(catkin REQUIRED)

catkin_package()

include(ExternalProject)

if("$ENV{ROS_DISTRO}" STRGREATER "melodic") # >= noetic
  set(CXXFLAGS_NOETIC "-Wno-error=deprecated-copy")
endif()
set(DOWNWARD_URL https://raw.githubusercontent.com/jsk-ros-pkg/archives/master/Fast-Downward-f33d3b65601f.tar.gz)
# ExternalProject tries the URLs in turn (CMake 3.7 and later): the same URL again, for a
# transient error of the network (e.g. of the CI)
if(NOT CMAKE_VERSION VERSION_LESS 3.7)
  set(DOWNWARD_URL ${DOWNWARD_URL} ${DOWNWARD_URL} ${DOWNWARD_URL})
endif()
# with JSK_3RDPARTY_DOWNLOAD_DIR (e.g. a cache of the CI), the file is downloaded there, and is not
# downloaded again while its hash matches
if(NOT "$ENV{JSK_3RDPARTY_DOWNLOAD_DIR}" STREQUAL "")
  set(DOWNWARD_DOWNLOAD_DIR DOWNLOAD_DIR $ENV{JSK_3RDPARTY_DOWNLOAD_DIR})
endif()
externalproject_add(downward
  # cdn.rawgit.com (and jsDelivr, where it redirects to) returns 404
  URL ${DOWNWARD_URL}
  ${DOWNWARD_DOWNLOAD_DIR}
  URL_MD5 0c882a28d4c0f60266f4baadf09e3ac7
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
