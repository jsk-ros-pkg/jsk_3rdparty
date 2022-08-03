#!/bin/bash

create_symlink () {
    ORIG_PATH=$1
    SYMLINK_PATH=$2
    if [ -e $SYMLINK_PATH ]; then
        echo "Symlink already exists: $SYMLINK_PATH"
    else
        ln -s $ORIG_PATH $SYMLINK_PATH
        echo "Create symlink $SYMLINK_PATH"
    fi
}

ARDUINO_VERSION=${1:-1.8.16}
ARDUINO_LIBRARY_PATH=$HOME/arduino-$ARDUINO_VERSION/libraries

# Create symlink for m5stack_ros/arduino_libraries
ORIG_PATH=$(rospack find m5stack_ros)/arduino_libraries
SYMLINK_PATH=$ARDUINO_LIBRARY_PATH/m5stack_ros
create_symlink $ORIG_PATH $SYMLINK_PATH
