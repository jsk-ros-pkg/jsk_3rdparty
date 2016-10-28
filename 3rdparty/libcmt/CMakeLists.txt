cmake_minimum_required(VERSION 2.8.3)
project(libcmt)
include(ExternalProject)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
execute_process(COMMAND
  git config --global http.sslVerify false)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
ExternalProject_Add(
  libcmt
  GIT_REPOSITORY https://github.com/delmottea/libCMT.git

  ## OLD TAG
  # GIT_TAG e4d7ea42edafe13b1070ef4d595b2d6062d79d1a

  ## RECENT TAG
  GIT_TAG 196c3ce4b4d9a85f71744ce1bd4f44631110e51d

  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  INSTALL_COMMAND echo "install"
  )

install(CODE "execute_process(COMMAND make install WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/libcmt-prefix/src/libcmt-build)")
