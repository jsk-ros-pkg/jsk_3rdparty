# AS-MLV-P2_Cal_Gas_Sensor

Publish [AS-MLV-P2 Cal_Gas_Sensor(Alcohols, aldehydes, ketones, organic acids, etc.)](https://modernroboticsinc.com/product-category/cal-sensors/?view=25).

## Overview

Published topics:

- `/cal_gas` (`std_msgs/UInt16`)

  Sensor analog output value

## Usage

- Follow [README.md](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros)

- Connect VCC(center) and GND(right) respectively, and connect analog output(left) to pin 36.

- Run

  ```bash
  roslaunch m5stack_ros m5stack_ros.launch
  ```
