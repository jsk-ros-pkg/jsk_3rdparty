if("$ENV{ROS_VERSION}" EQUAL 2)
  cmake_minimum_required(VERSION 3.8)
  project(jsk_power_switching_tools)

  # NOTE: auto_power_cycle.py stays ROS1-only for now: it subscribes to an
  # arbitrary topic with rospy.AnyMsg and calls the power cycle service
  # synchronously from a timer callback, which would deadlock a
  # single-threaded rclpy executor.
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)

  ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

  # The installed ROS2 copies get a python3 shebang (ROS2 base images ship
  # no bare `python`), keeping the source shared with ROS1.
  find_package(jsk_ros1_ros2_compat REQUIRED)
  jsk_ros1_ros2_compat_install_python(PROGRAMS node_scripts/usb_ppps_hub_driver.py node_scripts/usb_serial_troubleshooter_driver.py)

  install(DIRECTORY scripts udev
    DESTINATION share/${PROJECT_NAME}
    USE_SOURCE_PERMISSIONS
  )

  ament_package()

else() # if("$ENV{ROS_VERSION}" EQUAL 2)
cmake_minimum_required(VERSION 2.8.3)
project(jsk_power_switching_tools)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

file(GLOB NODE_SCRIPTS_PY node_scripts/*.py)
catkin_install_python(PROGRAMS ${NODE_SCRIPTS_PY}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/node_scripts
)

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN "*"
  PATTERN "*/*.py" EXCLUDE
)

install(DIRECTORY launch udev
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

endif() # if("$ENV{ROS_VERSION}" EQUAL 2)
