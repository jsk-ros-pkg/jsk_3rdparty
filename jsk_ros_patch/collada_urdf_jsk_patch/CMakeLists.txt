# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(collada_urdf_jsk_patch)

if(("$ENV{ROS_DISTRO}" STREQUAL "groovy") OR ("$ENV{ROS_DISTRO}" STREQUAL "hydro"))
  set(SOURCE_DISTRO hydro-devel)
  set(GIT_REPO robot_model)
elseif (("$ENV{ROS_DISTRO}" STREQUAL "indigo") OR ("$ENV{ROS_DISTRO}" STREQUAL "jade") OR ("$ENV{ROS_DISTRO}" STREQUAL "kinetic"))
  set(SOURCE_DISTRO bd4fc369d56eaa0c31d8cb17677e00b9d9685de6) # 1.11.13, before strip indigo
  set(GIT_REPO robot_model)
elseif (("$ENV{ROS_DISTRO}" STREQUAL "melodic") OR ("$ENV{ROS_DISTRO}" STREQUAL "noetic"))
  set(SOURCE_DISTRO 923c5d33bd245e82134e8ae02e4c9d379e80eb27) # 1.12.12
  set(GIT_REPO collada_urdf)
else()
  find_package(catkin REQUIRED)
  catkin_package(CATKIN_DEPENDS
    SKIP_CMAKE_CONFIG_GENERATION
    SKIP_PKG_CONFIG_GENERATION)
  file(TOUCH ${CMAKE_CURRENT_BINARY_DIR}/CATKIN_IGNORE)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/CATKIN_IGNORE  #catkin_lint: ignore_once missing_file
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  return()
endif()
if ("$ENV{ROS_DISTRO}" STREQUAL "kinetic" OR "$ENV{ROS_DISTRO}" STRGREATER "kinetic")
    set(CXXFLAGS CXXFLAGS=-std=gnu++11)
endif()

# find_package(catkin REQUIRED COMPONENTS collada_urdf)
# v5.0.0 have invalid inclduedir
# /usr/lib/x86_64-linux-gnu/pkgconfig/assimp.pc
#   includedir=/usr/../include/include
find_package(catkin REQUIRED)

catkin_package(CATKIN_DEPENDS collada_urdf)


#catkin_lint: ignore immutable_var
set(ENV{PKG_CONFIG_PATH} ${CATKIN_DEVEL_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH})
add_custom_target(urdf_to_collada ALL DEPENDS ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/urdf_to_collada)
add_custom_command(OUTPUT ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/urdf_to_collada
  DEPENDS Makefile
  COMMAND cmake -E chdir ${CMAKE_CURRENT_BINARY_DIR} make -f ${PROJECT_SOURCE_DIR}/Makefile PKG_CONFIG_PATH=${CATKIN_DEVEL_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH} PATCH_DIR=${PROJECT_SOURCE_DIR} SOURCE_DISTRO=${SOURCE_DISTRO} GIT_REPO=${GIT_REPO} ${CXXFLAGS}
  COMMAND cmake -E make_directory ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
  COMMAND cmake -E copy ${CMAKE_CURRENT_BINARY_DIR}/urdf_to_collada ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
  COMMAND cmake -E copy ${CMAKE_CURRENT_BINARY_DIR}/collada_to_urdf ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})

add_dependencies(urdf_to_collada libassimp_devel)

install(PROGRAMS ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/urdf_to_collada ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/collada_to_urdf
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})





