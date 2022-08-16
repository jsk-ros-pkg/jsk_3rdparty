#!/bin/bash

# Note that every header files (YYY.h) should be placed XXX directory
# XXX directory should be placed under $ARDUINO_LIBRARY_PATH
# For example, $ARDUINO_LIBRARY_PATH/XXX/YYY.h
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

# Create symlink for m5stack_ros/arduino_libraries
ORIG_PATH=$HOME/Arduino/libraries/M5Stack/examples/Unit/THERMAL_MLX90640
SYMLINK_PATH=$ARDUINO_LIBRARY_PATH/THERMAL_MLX90640
create_symlink $ORIG_PATH $SYMLINK_PATH

# Add Seeed-Studio/Grove_LED_Bar
git clone https://github.com/Seeed-Studio/Grove_LED_Bar.git $HOME/Arduino/libraries/Grove_LED_Bar
cd $HOME/Arduino/libraries/Grove_LED_Bar
git checkout -b b2964c4f9d967a0c891d25432cbc7ce83f3832ed
cd -
