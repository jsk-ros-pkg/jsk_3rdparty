cmake_minimum_required(VERSION 2.8.3)
project(libcmt)
include(ExternalProject)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
ExternalProject_Add(
  libcmt
  PREFIX ""
  GIT_REPOSITORY https://github.com/aginika/libCMT.git
  GIT_TAG add-make-install
  INSTALL_DIR ${CATKIN_DEVEL_PREFIX}
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
  )

catkin_package(
  )

