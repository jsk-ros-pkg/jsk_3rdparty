if("$ENV{ROS_VERSION}" EQUAL 2)
  cmake_minimum_required(VERSION 3.8)
  project(rospatlite)

  find_package(ament_cmake REQUIRED)

  # The installed ROS2 copies get a python3 shebang (ROS2 base images ship
  # no bare `python`), keeping the source shared with ROS1.
  find_package(jsk_ros1_ros2_compat REQUIRED)
  jsk_ros1_ros2_compat_install_python(PROGRAMS scripts/patlite_node.py scripts/patlite.py)

  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )

  ament_package()

else() # if("$ENV{ROS_VERSION}" EQUAL 2)
cmake_minimum_required(VERSION 2.8.3)
project(rospatlite)

find_package(catkin REQUIRED)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS std_msgs rospy
  )

install(DIRECTORY launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

endif() # if("$ENV{ROS_VERSION}" EQUAL 2)
