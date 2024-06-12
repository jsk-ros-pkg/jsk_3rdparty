cmake_minimum_required(VERSION 2.8.3)
project(bl_force_torque_sensor_ros)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  AmplifierOutputRawArray.msg
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)

file(GLOB SCRIPT_PROGRAMS node_scripts/*.py)
catkin_install_python(PROGRAMS ${SCRIPT_PROGRAMS}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/node_scripts
)

install(DIRECTORY arduino circuits config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
