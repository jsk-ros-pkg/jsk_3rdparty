cmake_minimum_required(VERSION 2.8.3)
project(osqp)

include(ExternalProject)

ExternalProject_Add(
  osqp
  GIT_REPOSITORY https://github.com/oxfordcontrol/osqp
  GIT_TAG v0.6.0
  GIT_SHALLOW     TRUE
  CMAKE_ARGS  -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  INSTALL_COMMAND echo "install"
  )

install(CODE "execute_process(COMMAND make install WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/osqp-prefix/src/osqp-build)")
