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

- Run

  ```bash
  roslaunch m5stack_ros m5stack_ros.launch
  ```

- In `ENV_III_with_battery` version, battery infomation (`battery_level` and `is_charging` rostopic) are published in addition to ENV III information.
