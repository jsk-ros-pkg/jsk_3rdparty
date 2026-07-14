# ENV III

Publish [ENV III](https://docs.m5stack.com/en/unit/envIII) temperature, humidity and atmospheric pressure sensor data.

## Dependencies

Install the following packages from Arduino libraries (Tools -> Manage Libraries)
- [M5Unit-ENV](https://github.com/m5stack/M5Unit-ENV/tree/0.0.5/src) version 0.0.5
- [Adafruit_BMP280](https://github.com/adafruit/Adafruit_BMP280_Library/tree/2.6.3) version 2.6.3

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

- Run

  ```bash
  roslaunch m5stack_ros m5stack_ros.launch
  ```
