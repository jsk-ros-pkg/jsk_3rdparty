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

set(VT_LIB_PATH /usr/vt/sayaka/M16/bin/x86_64/RAMIO/libvt_jpn.a)

if(EXISTS ${VT_LIB_PATH})
  include_directories(
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  )
  add_executable(voice_text src/voice_text.cpp)
  set_target_properties(voice_text PROPERTIES COMPILE_FLAGS -D_REENTRANT)
  target_link_libraries(voice_text
    ${catkin_LIBRARIES}
    ${VT_LIB_PATH} -lm -lpthread
  )
else()
  message(WARNING "VoiceText Library not found. Skip building binaries.")
endif()

install(PROGRAMS bin/text2wave
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
