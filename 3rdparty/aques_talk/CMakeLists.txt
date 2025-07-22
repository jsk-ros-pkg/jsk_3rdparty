cmake_minimum_required(VERSION 2.8.3)
project(aques_talk)

find_package(catkin REQUIRED)

catkin_package()

# Considering the use of docker in addition to the usual use,
# we use dpkg-architecture command instead of ${CMAKE_SYSTEM_PROCESSOR} to get the architecture.
# https://stackoverflow.com/a/58222507
# http://manpages.ubuntu.com/manpages/bionic/man1/dpkg-architecture.1.html
execute_process(COMMAND dpkg-architecture -qDEB_TARGET_GNU_CPU OUTPUT_VARIABLE ARCHITECTURE)
message(STATUS "DEB_TARGET_GNU_CPU : ${ARCHITECTURE}")
if("${ARCHITECTURE}" MATCHES x86_64* OR "${ARCHITECTURE}" STREQUAL "")
  if("${ARCHITECTURE}" STREQUAL "")
    message(WARNING "dpkg version is low, continue building as x86_64.")
  endif()
  set(AQTK2_LNX_LIB_DIR "lib64")
elseif("${ARCHITECTURE}" MATCHES i[0-9]86* )
  set(AQTK2_LNX_LIB_DIR "lib")
else()
  message(WARNING "${ARCHITECTURE} is not supported, do not compile aques_talk")
  return()
endif ()
message(STATUS "AQTK2_LNX_LIB_DIR : ${AQTK2_LNX_LIB_DIR}")

#
include(ExternalProject)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/download/src/)
set(LIBRARY_URL https://www.a-quest.com/archive/package/aqtk2-lnx-eva_230.zip)
file(DOWNLOAD
  ${LIBRARY_URL}
  ${CMAKE_CURRENT_BINARY_DIR}/download/src/aqtk2-lnx-eva_230.zip
  STATUS DOWNLOAD_STATUS
  LOG DOWNLOAD_LOG
  SHOW_PROGRESS
)
list(GET DOWNLOAD_STATUS 0 DOWNLOAD_CODE)
list(GET DOWNLOAD_STATUS 1 DOWNLOAD_MESSAGE)
if(NOT DOWNLOAD_CODE EQUAL 0)
  message(WARNING "Download failed with status: ${DOWNLOAD_CODE}")
  message(WARNING "Error message: ${DOWNLOAD_MESSAGE}")
  message(WARNING "Log: ${DOWNLOAD_LOG}")
else()
externalproject_add(aqtk2-lnx-eva
  PREFIX download
  URL ${LIBRARY_URL}
  URL_HASH MD5=4b0641066a826d0be7a016a162ab4043
  TIMEOUT 120
  CONFIGURE_COMMAND bash -c "echo configure"
  BUILD_COMMAND bash -c "(cd ${AQTK2_LNX_LIB_DIR} && ln -sf libAquesTalk2Eva.so.2.3 libAquesTalk2.so && ln -sf libAquesTalk2Eva.so.2.3 libAquesTalk2.so.2)"
  INSTALL_COMMAND bash -c "cp -P ${AQTK2_LNX_LIB_DIR}/libAquesTalk2* ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION} && mkdir -p ${CMAKE_CURRENT_SOURCE_DIR}/phont/ && cp phont/* ${CMAKE_CURRENT_SOURCE_DIR}/phont/"
  BUILD_IN_SOURCE 1
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/download/src/aqtk2-lnx-eva/${AQTK2_LNX_LIB_DIR})
link_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})
# compile SampleTalk
add_executable(SampleTalk src/SampleTalk.c)
target_link_libraries(SampleTalk AquesTalk2)

# Make compilation depend on our target:
add_dependencies(SampleTalk aqtk2-lnx-eva)
endif()

# install
catkin_install_python(PROGRAMS text2wave
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

if(CATKIN_ENABLE_TESTING AND DOWNLOAD_CODE EQUAL 0)
  find_package(catkin REQUIRED COMPONENTS rostest)
  if("${ARCHITECTURE}" MATCHES x86_64* OR "${ARCHITECTURE}" MATCHES i[0-9]86*)
    add_rostest(test/test_aques_talk.test)
  endif()
endif()
