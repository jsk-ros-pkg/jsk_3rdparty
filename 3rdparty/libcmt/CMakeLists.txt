cmake_minimum_required(VERSION 2.8.3)
project(libcmt)
include(ExternalProject)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# execute_process(COMMAND
#   git config --global http.sslVerify false)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
ExternalProject_Add(
  libcmt
  URL https://github.com/delmottea/libCMT/archive/e4d7ea42edafe13b1070ef4d595b2d6062d79d1a.zip
  PATCH_COMMAND patch -p1 < ${PROJECT_SOURCE_DIR}/fix_opencv3.patch
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  INSTALL_COMMAND echo "install"
  )

install(CODE "execute_process(COMMAND make install WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/libcmt-prefix/src/libcmt-build)")
