cmake_minimum_required(VERSION 2.8.3)
project(zdepth_image_transport)

find_package(catkin REQUIRED
  COMPONENTS
  image_transport
  message_generation
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
)

find_library(zdepth_LIB zdepth PATH_SUFFIXES zdepth)
find_library(zstd_LIB zstd PATH_SUFFIXES zdepth)
set(zdepth_LIBRARIES ${zdepth_LIB} ${zstd_LIB})
find_file(zdepth_INCLUDE zdepth.hpp PATH_SUFFIXES zdepth)
get_filename_component(zdepth_PATH ${zdepth_INCLUDE} DIRECTORY)
get_filename_component(zdepth_INCLUDE_DIRS ${zdepth_PATH} DIRECTORY)

add_message_files(
  DIRECTORY msg
  FILES
  ZDepthImage.msg
)

generate_messages(DEPENDENCIES std_msgs)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  message_runtime
  image_transport
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  DEPENDS
  zdepth
)

include_directories(include ${catkin_INCLUDE_DIRS} ${zdepth_INCLUDE_DIRS})

add_executable(zdepth_test src/test/zdepth_test.cpp)
target_link_libraries(zdepth_test ${catkin_LIBRARIES} ${zdepth_LIBRARIES})

add_library(${PROJECT_NAME} src/manifest.cpp src/zdepth_publisher.cpp src/zdepth_subscriber.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${zdepth_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

install(TARGETS zdepth_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES zdepth_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
