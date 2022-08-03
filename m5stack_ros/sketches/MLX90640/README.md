# MLX90640

Publish [MLX90640](https://www.sparkfun.com/products/14843) thermography data as ROS topic

## Overview

Communication:

- MLX90640 <-(I2C)-> M5Stack <-(Bluetooth/Wi-Fi/USB)-> PC
- You can use M5Stack's Grove connector for I2C connection

Published topics:

- `/thermal/image` (`sensor_msgs/Image`)

   thermography image calculated by MLX90640

- `/thermal/min_temp` (`std_msgs/Int16`)

   Minimum temperature in thermography image calculated by MLX90640

- `/thermal/max_temp` (`std_msgs/Int16`)

   Maximum temperature in thermography image calculated by MLX90640

- `/thermal/center_temp` (`std_msgs/Int16`)

   Center spot temperature in thermography image calculated by MLX90640

## Usage

- Follow [README.md](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros)

- Install [M5Stack THERMAL_MLX90640](https://github.com/m5stack/M5Stack/tree/0.3.6/examples/Unit/THERMAL_MLX90640) 3.6.0 version from Arduino library

- Run

  ```bash
  roslaunch m5stack_ros m5stack_ros.launch
  ```
