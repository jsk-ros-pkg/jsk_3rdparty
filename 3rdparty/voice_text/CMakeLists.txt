cmake_minimum_required(VERSION 2.8.3)
project(voice_text)
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  dynamic_reconfigure
  roscpp
  roslib
  message_generation)

find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(Poco REQUIRED COMPONENTS Foundation)

generate_dynamic_reconfigure_options(
  cfg/VoiceText.cfg
)

add_service_files(
  FILES
  TextToSpeech.srv
)

generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Poco_INCLUDE_DIRS}
)
add_executable(voice_text src/voice_text.cpp)
set_target_properties(voice_text
  PROPERTIES COMPILE_FLAGS -D_REENTRANT)
target_link_libraries(voice_text
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Poco_LIBRARIES}
  -lm -lpthread
)

install(PROGRAMS bin/text2wave
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY data launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/voice_text.test)
endif()
