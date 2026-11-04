if("$ENV{ROS_VERSION}" EQUAL 2)
  cmake_minimum_required(VERSION 3.8)
  project(influxdb_store)

  # NOTE: only joint_states_logger.py/network_states_logger.py are ported to
  # ROS2. The transform loggers rely on the ROS1 tf2_ros buffer_server's
  # ~tf2_frames service and rospy.TimerEvent, and the battery loggers on
  # pr2_msgs/power_msgs, so they stay ROS1-only for now.
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)

  ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})

  # The installed ROS2 copies get a python3 shebang (ROS2 base images ship
  # no bare `python`), keeping the source shared with ROS1.
  find_package(jsk_ros1_ros2_compat REQUIRED)
  jsk_ros1_ros2_compat_install_python(PROGRAMS node_scripts/joint_states_logger.py node_scripts/network_states_logger.py)

  ament_package()

else() # if("$ENV{ROS_VERSION}" EQUAL 2)
cmake_minimum_required(VERSION 2.8.3)
project(influxdb_store)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

file(GLOB NODE_SCRIPTS_FILES node_scripts/*)
catkin_install_python(
  PROGRAMS ${NODE_SCRIPTS_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch sample
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

endif() # if("$ENV{ROS_VERSION}" EQUAL 2)
