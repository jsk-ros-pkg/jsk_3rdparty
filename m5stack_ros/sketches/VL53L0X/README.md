# VL53L0X

Publish [VL53L0X](https://www.adafruit.com/product/3317) ToF data as ROS topic

## Dependencies

Install the following packages from Arduino libraries (Tools -> Manage Libraries)
- [VL53L0X by Pololu](https://github.com/pololu/vl53l0x-arduino/tree/1.3.0) version 1.3.0

## Overview

Communication:

- VL53L0X <-(I2C)-> M5Stack <-(Bluetooth/Wi-Fi/USB)-> PC
- You can use M5Stack's Grove connector for I2C connection

Published topics:

- `/tof` (`std_msgs/UInt16`)

  Distance [mm] calculated by VL53L0X, ToF proximity sensor

## Usage

- Follow [README.md](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros)

- Run

  ```bash
  roslaunch m5stack_ros m5stack_ros.launch
  ```

# Trouble Shooting
  - If the retunred value is 8190 or 8191, it might be our-of-range error.

     https://github.com/pololu/vl53l0x-arduino/issues/28

  - If sensing does not started, reset the internal state of VL53L0X:

    - Make sure that M5Stack and VL53L0X are connected with cable
    - Unplug the USB cable from M5Stack
    - Double-click the main button to sleep M5Stack
    - Unplug and plug the cable between M5Stack and VL53L0X to reset VL53L0X
    - Plug the USB cable to M5Stack or click the main button to boot the M5Stack
