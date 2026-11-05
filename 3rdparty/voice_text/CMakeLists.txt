cmake_minimum_required(VERSION 2.8.3)
project(voice_text)

# voice_text requires C++ 11
if(CMAKE_VERSION VERSION_LESS "3.1")
  if(NOT CMAKE_CXX_FLAGS MATCHES "-std=")
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag("-std=c++11" HAS_CXX11_FLAG)
    if(HAS_CXX11_FLAG)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    else()
      check_cxx_compiler_flag("-std=gnu++11" HAS_GNUXX11_FLAG)
      if(HAS_GNUXX11_FLAG)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
      endif()
    endif()
  endif()
endif()
if("$ENV{ROS_DISTRO}" STREQUAL "kinetic")
  add_compile_options(-std=c++11)
endif()

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

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(voice_text src/voice_text.cpp src/vt_handler.cpp)
add_dependencies(voice_text ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)
set_target_properties(voice_text PROPERTIES COMPILE_FLAGS -D_REENTRANT)

target_link_libraries(voice_text
    ${catkin_LIBRARIES} -lm -lpthread -ldl
)

install(TARGETS voice_text # do not install vt_dummy target, that should be installed from voice_text library
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/text2wave.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS bin/text2wave
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/bin
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
