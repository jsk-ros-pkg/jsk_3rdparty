cmake_minimum_required(VERSION 2.8.3)
project(voice_text)
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  message_generation)

find_package(Boost REQUIRED COMPONENTS filesystem)

generate_dynamic_reconfigure_options(
  cfg/VoiceText.cfg
)

add_service_files(
  FILES
  TextToSpeech.srv
)

generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)

# find voice text headers and libraries
file(GLOB VT_ROOT /usr/vt/*/*)
if(NOT VT_ROOT)
  message(WARNING "VoiceText directory should be /usr/vt/*/* (e.g., /usr/vt/sayaka/M16) but it is not found")
  set(VT_ROOT /usr/vt/sayaka/M16)  # default value for following configure_file
else()
  if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "4")
    set(VT_LIB_PATH_OLD ${VT_ROOT}/bin/x86_32/RAMIO/libvt_jpn.so)  # e.g., /usr/vt/sayaka/M16/bin/x86_32/RAMIO/libvt_jpn.so
    set(VT_LIB_PATH_NEW ${VT_ROOT}/bin/LINUX32_GLIBC3/RAMIO/libvt_jpn.so)  # e.g., /usr/vt/risa/H16/bin/LINUX32_GLIBC3/RAMIO/libvt_jpn.so
    set(VT_LIB_PATH_FILEIO ${VT_ROOT}/bin/FILEIO/LINUX32_GLIBC3/libvt_jpn.so)  # e.g., /usr/vt/hikari/D16/bin/FILEIO/LINUX32_GLIBC3/libvt_jpn.so
  elseif("${CMAKE_SIZEOF_VOID_P}" STREQUAL "8")
    set(VT_LIB_PATH_OLD ${VT_ROOT}/bin/x86_64/RAMIO/libvt_jpn.so)  # e.g., /usr/vt/sayaka/M16/bin/x86_64/RAMIO/libvt_jpn.so
    set(VT_LIB_PATH_NEW ${VT_ROOT}/bin/LINUX64_GLIBC3/RAMIO/libvt_jpn.so)  # e.g., /usr/vt/risa/H16/bin/LINUX64_GLIBC3/RAMIO/libvt_jpn.so
    set(VT_LIB_PATH_FILEIO ${VT_ROOT}/bin/FILEIO/LINUX64_GLIBC3/libvt_jpn.so)  # e.g., /usr/vt/hikari/D16/bin/FILEIO/LINUX64_GLIBC3/libvt_jpn.so
  endif()
  if(EXISTS ${VT_LIB_PATH_OLD})
    set(VT_LIB_PATH ${VT_LIB_PATH_OLD})
  elseif(EXISTS ${VT_LIB_PATH_NEW})
    set(VT_LIB_PATH ${VT_LIB_PATH_NEW})
  elseif(EXISTS ${VT_LIB_PATH_FILEIO})
    set(VT_LIB_PATH ${VT_LIB_PATH_FILEIO})
  endif()
  if(VT_LIB_PATH)
    message(WARNING "VoiceText Engine library is found at ${VT_LIB_PATH}")
  else()
    message(WARNING "VoiceText Engine library is not found at ${VT_LIB_PATH_OLD} or ${VT_LIB_PATH_NEW} or ${VT_LIB_PATH_FILE}")
  endif()
endif()

# use traditional sdk or new api
set(VT_SDK_H ${VT_ROOT}/inc/vt_jpn.h)
set(VT_API_H ${VT_ROOT}/inc/vtapi.h)
if(EXISTS ${VT_SDK_H})
  # use traditional sdk
  message(WARNING "Using SDK because ${VT_SDK_H} was found.")
  set(VT_INCLUDE ${VT_SDK_H})
elseif(EXISTS ${VT_API_H})
  # use new api
  message(WARNING "Using API because ${VT_SDK_H} was not found but ${VT_API_H} was found.")
  string(REPLACE "/" ";" _VT_ROOT_SPLIT ${VT_ROOT})
  list(GET _VT_ROOT_SPLIT 3 VT_SPEAKER)
  list(GET _VT_ROOT_SPLIT 4 VT_TYPE)
  set(VT_API_LIB_PATH ${VT_ROOT}/bin/libvtapi.so)
  set(VT_INCLUDE ${VT_API_H})
  set(VT_BIN ${VT_ROOT}/bin)
endif()

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
add_executable(voice_text src/voice_text.cpp)
add_dependencies(voice_text ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)
set_target_properties(voice_text PROPERTIES COMPILE_FLAGS -D_REENTRANT)

# build with voice_text engint or dummy
if(VT_LIB_PATH)
  message(WARNING "Building with Voice Text library")
  set(BUILD_FLAGS "-DVT_ROOT=${VT_ROOT} -DVT_INCLUDE=${VT_INCLUDE} -DVT_SPEAKER=${VT_SPEAKER} -DVT_TYPE=${VT_TYPE}")
  set_target_properties(voice_text PROPERTIES COMPILE_OPTIONS ${BUILD_FLAGS})
else()
  message(WARNING "Building dummy library")
  add_library(vt_dummy src/dummy/vt_dummy.cpp)
  set_target_properties(vt_dummy PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
  set_target_properties(vt_dummy PROPERTIES LIBRARY_OUTPUT_NAME vt_jpn)
  set_target_properties(voice_text PROPERTIES COMPILE_FLAGS -DUSE_DUMMY_INCLUDE)
  set(VT_LIB_PATH ${PROJECT_BINARY_DIR}/libvt_jpn.so)
  add_dependencies(voice_text vt_dummy)
endif()

target_link_libraries(voice_text
    ${catkin_LIBRARIES}
    ${VT_LIB_PATH} -lm -lpthread
    ${VT_API_LIB_PATH} -lm -lpthread -ldl
)

install(TARGETS voice_text # do not install vt_dummy target, that should be installed from voice_text library
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS bin/text2wave
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/bin
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
