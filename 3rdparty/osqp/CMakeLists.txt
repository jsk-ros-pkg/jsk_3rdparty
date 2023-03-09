cmake_minimum_required(VERSION 2.8.3)
project(osqp)

include(ExternalProject)

ExternalProject_Add(
  osqp
  URL https://github.com/osqp/osqp/releases/download/v0.6.2/complete_sources.tar.gz
  URL_HASH MD5=66738e622174eb506e0cee106891397e
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  INSTALL_COMMAND echo "install"
  )

install(CODE "execute_process(COMMAND make install WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/osqp-prefix/src/osqp-build)")
