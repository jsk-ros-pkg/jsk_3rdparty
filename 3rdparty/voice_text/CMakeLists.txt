cmake_minimum_required(VERSION 2.8.3)
project(voice_text)
find_package(catkin REQUIRED)

catkin_package()

if(EXISTS /usr/vt/sayaka/M16/bin/x86_64/RAMIO/libvt_jpn.a)
  add_executable(voicetext src/voicetext.c)
  set_target_properties(voicetext PROPERTIES COMPILE_FLAGS -D_REENTRANT)
  target_link_libraries(voicetext /usr/vt/sayaka/M16/bin/x86_64/RAMIO/libvt_jpn.a -lm -lpthread)
endif()