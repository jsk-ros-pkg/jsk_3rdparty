# TVOC_SGP30

Publish [TVOC_SGP30](https://shop.m5stack.com/products/tvoc-eco2-gas-unit-sgp30) air data as ROS topic

## Overview

Communication:

- TVOC_SGP30 <-(I2C)-> M5Stack <-(Bluetooth/Wi-Fi/USB)-> PC
- You can use M5Stack's Grove connector for I2C connection

Published topics:

- `tvoc` (`std_msgs/UInt16`)

   TVOC concentration

- `eco2` (`std_msgs/UInt16`)

   eCO2 concentration

- `sound_play/goal` (`sound_play/SoundRequestActionGoal`)

   If the co2 concentration exceeds 1000ppm, the robot will alert you by voice.

## Usage

- Follow [README.md](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros)

- Run

  ```bash
  roslaunch m5stack_ros m5stack_ros.launch
  ```

- In `MPU9250_with_battery` version, battery infomation (`battery_level` and `is_charging` rostopic) are published in addition to IMU information.
