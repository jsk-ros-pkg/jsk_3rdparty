cmake_minimum_required(VERSION 2.8.3)
project(osqp)

include(ExternalProject)

if("$ENV{ROS_DISTRO}" STRGREATER "indigo")
  set(OSQP_URL https://github.com/osqp/osqp/releases/download/v0.6.2/complete_sources.tar.gz)
  # ExternalProject tries the URLs in turn (CMake 3.7 and later): the same URL again, for a
  # transient error of the network (e.g. of the CI)
  if(NOT CMAKE_VERSION VERSION_LESS 3.7)
    set(OSQP_URL ${OSQP_URL} ${OSQP_URL} ${OSQP_URL})
  endif()
  # with JSK_3RDPARTY_DOWNLOAD_DIR (e.g. a cache of the CI), the file is downloaded there, and is not
  # downloaded again while its hash matches
  if(NOT "$ENV{JSK_3RDPARTY_DOWNLOAD_DIR}" STREQUAL "")
    set(OSQP_DOWNLOAD_DIR DOWNLOAD_DIR $ENV{JSK_3RDPARTY_DOWNLOAD_DIR})
  endif()
  ExternalProject_Add(
    osqp
    URL ${OSQP_URL}
    ${OSQP_DOWNLOAD_DIR}
    URL_HASH MD5=66738e622174eb506e0cee106891397e
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    INSTALL_COMMAND echo "install"
    )

  install(CODE "execute_process(COMMAND make install WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/osqp-prefix/src/osqp-build)")
else()
  install(CODE "message(WARNING \"Skipping osqp install because ROS is too old\")")
endif()
