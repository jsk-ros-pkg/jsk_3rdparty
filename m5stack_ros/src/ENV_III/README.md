# ENV III

Publish [ENV III](https://docs.m5stack.com/en/unit/envIII) temperature, humidity and atmospheric pressure sensor data.

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
