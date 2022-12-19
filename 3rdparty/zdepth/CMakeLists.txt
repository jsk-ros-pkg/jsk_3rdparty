cmake_minimum_required(VERSION 2.8.3)
project(zdepth)

include(ExternalProject)

ExternalProject_Add(
  zdepth
  GIT_REPOSITORY  https://github.com/catid/Zdepth.git
  GIT_TAG         ac7c6d8e944d07be2404e5a1eaa04562595f3756
  GIT_SHALLOW     TRUE
  PATCH_COMMAND cat ${PROJECT_SOURCE_DIR}/fix_cmakelists.patch | patch -p1
  CMAKE_ARGS      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  INSTALL_COMMAND echo "install"
 )


install(CODE "execute_process(COMMAND make install WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/zdepth-prefix/src/zdepth-build)")
