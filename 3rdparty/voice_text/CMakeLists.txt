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

file(GLOB VT_ROOT /usr/vt/*/*)
if(NOT VT_ROOT)
  message(WARNING "VoiceText directory should be /usr/vt/*/* (e.g., /usr/vt/sayaka/M16) but is not found")
  set(VT_ROOT /usr/vt/sayaka/M16)  # default value for following configure_file
else()
  if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "4")
    set(VT_LIB_PATH_OLD ${VT_ROOT}/bin/x86_32/RAMIO/libvt_jpn.so)  # e.g., /usr/vt/sayaka/M16/bin/x86_32/RAMIO/libvt_jpn.so
    set(VT_LIB_PATH_NEW ${VT_ROOT}/bin/LINUX32_GLIBC3/RAMIO/libvt_jpn.so)  # e.g., /usr/vt/risa/H16/bin/LINUX32_GLIBC3/RAMIO/libvt_jpn.so
  elseif("${CMAKE_SIZEOF_VOID_P}" STREQUAL "8")
    set(VT_LIB_PATH_OLD ${VT_ROOT}/bin/x86_64/RAMIO/libvt_jpn.so)  # e.g., /usr/vt/sayaka/M16/bin/x86_64/RAMIO/libvt_jpn.so
    set(VT_LIB_PATH_NEW ${VT_ROOT}/bin/LINUX64_GLIBC3/RAMIO/libvt_jpn.so)  # e.g., /usr/vt/risa/H16/bin/LINUX64_GLIBC3/RAMIO/libvt_jpn.so
  endif()
  if(EXISTS ${VT_LIB_PATH_OLD})
    set(VT_LIB_PATH ${VT_LIB_PATH_OLD})
  else()
    if(EXISTS ${VT_LIB_PATH_NEW})
      set(VT_LIB_PATH ${VT_LIB_PATH_NEW})
    endif()
  endif()
  if(VT_LIB_PATH)
    message(WARNING "VoiceText library is found at ${VT_LIB_PATH}")
  else()
    message(WARNING "VoiceText library is not found at ${VT_LIB_PATH_OLD} or ${VT_LIB_PATH_NEW}")
  endif()
endif()
configure_file(src/voice_text.cpp.in ${PROJECT_SOURCE_DIR}/src/voice_text.cpp)

  include_directories(
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  )
  add_executable(voice_text src/voice_text.cpp)
  add_dependencies(voice_text ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)
  set_target_properties(voice_text PROPERTIES COMPILE_FLAGS -D_REENTRANT)

if(NOT VT_LIB_PATH)
  message(WARNING "Building dummy library")
  add_library(vt_dummy src/dummy/vt_dummy.cpp)
  set_target_properties(vt_dummy PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
  set_target_properties(vt_dummy PROPERTIES LIBRARY_OUTPUT_NAME vt_jpn)
  set_target_properties(voice_text PROPERTIES COMPILE_FLAGS -DUSE_DUMMY_INCLUDE)
  set(VT_LIB_PATH ${PROJECT_BINARY_DIR}/libvt_jpn.so)
endif()

  target_link_libraries(voice_text
    ${catkin_LIBRARIES}
    ${VT_LIB_PATH} -lm -lpthread
  )
if(NOT EXISTS ${VT_LIB_PATH})
  add_dependencies(voice_text vt_dummy)
endif()

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
