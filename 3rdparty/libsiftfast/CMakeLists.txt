cmake_minimum_required(VERSION 2.8.3)
project(libsiftfast)

find_package(catkin)

add_custom_target(libsiftfast ALL
  DEPENDS ${CATKIN_DEVEL_PREFIX}/lib/libsiftfast.so)
add_custom_command(OUTPUT ${CATKIN_DEVEL_PREFIX}/lib/libsiftfast.so
  DEPENDS Makefile
  COMMAND  make -f ${PROJECT_SOURCE_DIR}/Makefile SOURCE_DIR=${PROJECT_SOURCE_DIR} INSTALL_DIR=${CATKIN_DEVEL_PREFIX} PYTHON_INSTALL_DIR=${PYTHON_INSTALL_DIR})

# force install directory for catkin_package
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)
if("$ENV{ROS_DISTRO}" STREQUAL "groovy")
  catkin_package(
    INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
    LIBRARIES    siftfast
    SKIP_CMAKE_CONFIG_GENERATION
    )
  set(${PROJECT_NAME}_EXPORTED_TARGETS libsiftfast)
else()
  catkin_package(
    INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
    LIBRARIES    siftfast
    EXPORTED_TARGETS libsiftfast
    )
endif()

install(PROGRAMS ${CATKIN_DEVEL_PREFIX}/bin/siftfast
        DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(CODE "
  FILE(INSTALL DESTINATION ${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR} TYPE PROGRAM FILES ${CATKIN_DEVEL_PREFIX}/${PYTHON_INSTALL_DIR}/siftfastpy.so)
")

install(CODE "
  file(GLOB _libs ${CATKIN_DEVEL_PREFIX}/lib/libsiftfast.*)
  foreach(_lib \${_libs})
    FILE(INSTALL DESTINATION \"\${CMAKE_INSTALL_PREFIX}/lib\" TYPE PROGRAM FILES \"\${_lib}\" )
  endforeach()
")

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/siftfast
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/share/siftfast/
        DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/libsiftfast
        USE_SOURCE_PERMISSIONS)

# only for deb packaging...
if ($ENV{DH_OPTIONS}) # deb package need to remove generated files
  install(CODE "execute_process(COMMAND cmake -E remove_directory ${CATKIN_DEVEL_PREFIX}")
endif()



