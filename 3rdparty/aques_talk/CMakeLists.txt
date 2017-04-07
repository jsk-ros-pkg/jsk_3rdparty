cmake_minimum_required(VERSION 2.8.3)
project(aques_talk)

find_package(catkin REQUIRED)

catkin_package()

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64* )
set(AQTK2_LNX_LIB_DIR "lib64")
else ()
set(AQTK2_LNX_LIB_DIR "lib")
endif ()

file(DOWNLOAD http://www.a-quest.com/download/package/aqtk2-lnx-eva_210.zip
  ${CMAKE_CURRENT_BINARY_DIR}/aqtk2-lnx-eva_210.zip
  EXPECTED_MD5 68029448ecf4d36337e754c6eb3bb5d8)
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/include/AquesTalk2.h
  COMMAND cd ${CMAKE_CURRENT_BINARY_DIR} && 
  unzip -u aqtk2-lnx-eva_210.zip -d build &&
  mkdir -p include lib phont &&
  cp build/aqtk2-lnx-eva/${AQTK2_LNX_LIB_DIR}/*.h include &&
  cp build/aqtk2-lnx-eva/${AQTK2_LNX_LIB_DIR}/libAquesTalk2Eva.so.2.1 lib/libAquesTalk2.so.2 &&
  (cd lib && ln -sf libAquesTalk2.so.2 libAquesTalk2.so ) &&
  cp -r build/aqtk2-lnx-eva/phont ./ &&
  cp -r build/aqtk2-lnx-eva/phont ${PROJECT_SOURCE_DIR}
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/aqtk2-lnx-eva_210.zip
  )

link_directories(${CMAKE_CURRENT_BINARY_DIR}/lib)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)
add_executable(SampleTalk src/SampleTalk.c)
target_link_libraries(SampleTalk AquesTalk2)
add_custom_target(include_target DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/include/AquesTalk2.h)
add_dependencies(SampleTalk include_target)

