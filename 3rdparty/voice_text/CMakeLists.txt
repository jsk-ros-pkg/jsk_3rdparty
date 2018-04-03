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

if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "4")
  set(VT_LIB_ARCH x86_32/)
elseif("${CMAKE_SIZEOF_VOID_P}" STREQUAL "8")
  set(VT_LIB_ARCH x86_64/)
endif()
set(VT_LIB_PATH /usr/vt/sayaka/M16/bin/${VT_LIB_ARCH}RAMIO/libvt_jpn.so)

  include_directories(
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  )
  add_executable(voice_text src/voice_text.cpp)
  add_dependencies(voice_text ${PROJECT_NAME}_generate_messages_cpp)
  set_target_properties(voice_text PROPERTIES COMPILE_FLAGS -D_REENTRANT)

if(NOT EXISTS ${VT_LIB_PATH})
  message(WARNING "VoiceText Library not found at ${VT_LIB_PATH}. building dummy library.")
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

install(TARGETS voice_text # do not install vt_dummy target, that should be installed from voice_text library
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS bin/text2wave
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
