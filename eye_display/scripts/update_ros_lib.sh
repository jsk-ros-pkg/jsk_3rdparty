#!/bin/bash

CURRENT_DIR=$(pwd)

# If returned value of rospack find eye_display is not zero, please source devel/setup.bash first
rospack find eye_display >/dev/null 2>&1
if [ $? -ne 0 ]; then
    echo "Please source setup.bash first"
    cd $CURRENT_DIR
    exit 1
fi

TMP_DIR=/tmp/eye_display_ros_lib
TARGET_DIR=$(rospack find eye_display)/lib/

cd $TARGET_DIR
rm -rf ros_lib && mkdir ros_lib
mkdir -p $TMP_DIR && cd $TMP_DIR
rosrun rosserial_arduino make_libraries.py .
cp -r ros_lib/ros $TARGET_DIR/ros_lib/
cp -r ros_lib/*.h $TARGET_DIR/ros_lib/
cp -r ros_lib/*.cpp $TARGET_DIR/ros_lib/
cp -r ros_lib/std_msgs $TARGET_DIR/ros_lib/
cp -r ros_lib/rosserial_msgs $TARGET_DIR/ros_lib/
cp -r ros_lib/std_srvs $TARGET_DIR/ros_lib/
cp -r ros_lib/geometry_msgs $TARGET_DIR/ros_lib/
cp -r ros_lib/eye_display $TARGET_DIR/ros_lib/

cd $CURRENT_DIR
