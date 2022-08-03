# ENV III

Publish [ENV III](https://docs.m5stack.com/en/unit/envIII) temperature, humidity and atmospheric pressure sensor data.

## Dependencies

- [M5Unit-ENV](https://github.com/m5stack/M5Unit-ENV)
- [Adafruit BMP280 Driver](https://github.com/adafruit/Adafruit_BMP280_Library)

## Overview

Published topics:

- `/temperature` (`std_msgs/Float32`)

  Temperature [C]

- `/humidity` (`std_msgs/Float32`)

  Humidity [%]

- `/pressure` (`std_msgs/Float32`)

  Pressure [Pa]

## Usage

- Follow [README.md](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros)

- Required libraries are
  - [M5Unit-ENV](https://github.com/m5stack/M5Unit-ENV/tree/0.0.5/src) version 0.0.5 from Arduino library
    https://github.com/m5stack/M5Unit-ENV
  - [Adafruit_BMP280](https://github.com/adafruit/Adafruit_BMP280_Library/tree/2.6.3) version 2.6.3 from Arduino library

- Run

  ```bash
  roslaunch m5stack_ros m5stack_ros.launch
  ```
