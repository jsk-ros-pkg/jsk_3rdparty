cmake_minimum_required(VERSION 2.8.3)
project(m5stack_ros)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

file(GLOB NODE_SCRIPTS_FILES node_scripts/*)
catkin_install_python(
  PROGRAMS ${NODE_SCRIPTS_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories()

# Do not test with CI at this time.
# This is because Arduino IDE setup is not automatic.
# The following compile test should be done after manual setup.

# if(CATKIN_ENABLE_TESTING)
#   find_package(catkin REQUIRED COMPONENTS rostest)
#   # Specify arduino executable with which you have done your setup
#   set(arduino_path $ENV{HOME}/arduino-1.8.16/arduino)
#   message("Test arduino sketches with ${arduino_path}")
#   # Find all sketches under sketches directory
#   file(GLOB sketch_dirs ${PROJECT_SOURCE_DIR}/sketches/*)
#   foreach(sketch_dir ${sketch_dirs})
#     get_filename_component(sketch_name ${sketch_dir} NAME)
#     message("Test compiling ${sketch_path}")
#     # Sketch compile test
#     add_rostest(test/test_sketch.test ARGS
#       arduino_path:=${arduino_path}
#       sketch_name:=${sketch_name})
#   endforeach()
# endif()
