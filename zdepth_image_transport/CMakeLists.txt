if("$ENV{ROS_VERSION}" EQUAL 2)
  cmake_minimum_required(VERSION 3.8)
  project(zdepth_image_transport)

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(image_transport REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)

  # libzdepth/libzstd and zdepth/zdepth.hpp of the zdepth package, as ROS1
  find_library(${PROJECT_NAME}_zdepth_LIB zdepth PATH_SUFFIXES zdepth REQUIRED)
  find_library(${PROJECT_NAME}_zstd_LIB zstd PATH_SUFFIXES zdepth NO_CMAKE_SYSTEM_PATH REQUIRED)
  set(${PROJECT_NAME}_zdepth_LIBRARIES ${${PROJECT_NAME}_zdepth_LIB} ${${PROJECT_NAME}_zstd_LIB})
  find_path(${PROJECT_NAME}_zdepth_INCLUDE_DIRS zdepth/zdepth.hpp REQUIRED)

  # the target ${PROJECT_NAME} is the plugin library (libzdepth_image_transport.so)
  rosidl_generate_interfaces(${PROJECT_NAME}_msgs
    msg/ZDepthImage.msg
    DEPENDENCIES std_msgs
    LIBRARY_NAME ${PROJECT_NAME}
  )
  rosidl_get_typesupport_target(${PROJECT_NAME}_cpp_typesupport ${PROJECT_NAME}_msgs rosidl_typesupport_cpp)

  add_executable(zdepth_test src/test/zdepth_test.cpp)
  target_include_directories(zdepth_test PRIVATE ${${PROJECT_NAME}_zdepth_INCLUDE_DIRS})
  target_link_libraries(zdepth_test ${${PROJECT_NAME}_zdepth_LIBRARIES})

  add_library(${PROJECT_NAME} SHARED src/manifest.cpp src/zdepth_publisher.cpp src/zdepth_subscriber.cpp)
  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${${PROJECT_NAME}_zdepth_INCLUDE_DIRS}
  )
  target_link_libraries(${PROJECT_NAME}
    ${${PROJECT_NAME}_cpp_typesupport}
    image_transport::image_transport
    pluginlib::pluginlib
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    ${${PROJECT_NAME}_zdepth_LIBRARIES}
  )
  # the library path of zdepth_plugins.xml (lib/libzdepth_image_transport)
  # is not found by the pluginlib of ROS2, which takes the library name
  pluginlib_export_plugin_description_file(image_transport zdepth_plugins_ros2.xml)

  install(TARGETS zdepth_test
    DESTINATION lib/${PROJECT_NAME}
  )
  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  install(DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
  )

  ament_export_dependencies(rosidl_default_runtime image_transport pluginlib rclcpp sensor_msgs std_msgs)
  ament_package()

else() # if("$ENV{ROS_VERSION}" EQUAL 2)
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

if("$ENV{ROS_DISTRO}" STRGREATER "kinetic")
  find_library(${PROJECT_NAME}_zdepth_LIB zdepth PATH_SUFFIXES zdepth)
  find_library(${PROJECT_NAME}_zstd_LIB zstd PATH_SUFFIXES zdepth)
  set(${PROJECT_NAME}_zdepth_LIBRARIES ${${PROJECT_NAME}_zdepth_LIB} ${${PROJECT_NAME}_zstd_LIB})
  find_file(${PROJECT_NAME}_zdepth_INCLUDE zdepth.hpp PATH_SUFFIXES zdepth)
  get_filename_component(${PROJECT_NAME}_zdepth_PATH ${${PROJECT_NAME}_zdepth_INCLUDE} DIRECTORY)
  get_filename_component(${PROJECT_NAME}_zdepth_INCLUDE_DIRS ${${PROJECT_NAME}_zdepth_PATH} DIRECTORY)
endif()

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
  ${PROJECT_NAME}_zdepth
)

if("$ENV{ROS_DISTRO}" STRGREATER "kinetic")
  include_directories(include ${catkin_INCLUDE_DIRS} ${${PROJECT_NAME}_zdepth_INCLUDE_DIRS})

  add_executable(zdepth_test src/test/zdepth_test.cpp)
  target_link_libraries(zdepth_test ${catkin_LIBRARIES} ${${PROJECT_NAME}_zdepth_LIBRARIES})

  add_library(${PROJECT_NAME} src/manifest.cpp src/zdepth_publisher.cpp src/zdepth_subscriber.cpp)
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${${PROJECT_NAME}_zdepth_LIBRARIES})
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
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES zdepth_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

endif() # if("$ENV{ROS_VERSION}" EQUAL 2)
