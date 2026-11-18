# VCNL4040

Publish [VCNL4040](https://www.adafruit.com/product/4161) proximity data as ROS topic

## Overview

Communication:

- VCNL4040 <-(I2C)-> M5Stack <-(Bluetooth/Wi-Fi/USB)-> PC
- You can use M5Stack's Grove connector for I2C connection

Published topics:

- `/proximity` (`std_msgs/UInt16`)

  IR intensity calculated by VCNL4040 proximity sensor

## Usage

- Follow [README.md](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros)

- Run

  ```bash
  roslaunch m5stack_ros m5stack_ros.launch
  ```
