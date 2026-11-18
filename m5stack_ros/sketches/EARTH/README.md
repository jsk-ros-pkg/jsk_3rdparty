# EARTH

Publish [EARTH](https://shop.m5stack.com/products/earth-sensor-unit) soil moisture value

## Overview

Published topics:

- `/moist` (`std_msgs/Bool`)

  Whether the target is moist.

- `/moisture` (`std_msgs/Bool`)

  Moisture content. 0: Most moist. 4095: least moist.

## Usage

- Follow [README.md](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros)

- Run

  ```bash
  roslaunch m5stack_ros m5stack_ros.launch
  ```

- Note that `M5StickC` and `M5StickC Plus` is tested.
