cmake_minimum_required(VERSION 2.8.3)
project(slic)
include(ExternalProject)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# execute_process(COMMAND
#   git config --global http.sslVerify false)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
ExternalProject_Add(
  slic
  URL https://github.com/garaemon/SLIC-Superpixels/archive/78d9a2ba7ae1d3fee8c2ec774a52536c5f08f07c.zip
  GIT_TAG 78d9a2ba7ae1d3fee8c2ec774a52536c5f08f07c
  CMAKE_COMMAND echo "configure"
  BUILD_COMMAND echo "build"
  INSTALL_COMMAND echo "install"
  )

find_package( OpenCV REQUIRED )
install(CODE "execute_process(COMMAND cmake -E make_directory ${CMAKE_INSTALL_PREFIX}/include/)")
install(CODE "execute_process(COMMAND cmake -E copy slic.h ${CMAKE_INSTALL_PREFIX}/include/ WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/slic-prefix/src/slic/)")

install(CODE "execute_process(COMMAND cmake -E make_directory ${CMAKE_INSTALL_PREFIX}/lib/)")
install(CODE "execute_process(COMMAND gcc -I${OpenCV_INCLUDE_DIRS} -fPIC --shared -o ${CMAKE_INSTALL_PREFIX}/lib/libslic.so ${CMAKE_CURRENT_BINARY_DIR}/slic-prefix/src/slic/slic.cpp)")


