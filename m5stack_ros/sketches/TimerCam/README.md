# TimerCam

Publish [TimerCam](https://docs.m5stack.com/en/unit/timercam_f) fisheye image

## Dependencies

- [Timer-CAM](https://github.com/m5stack/TimerCam-arduino/tree/0.0.2) version 0.0.2
  - NOTE that **compile will fail without [this change](https://github.com/m5stack/TimerCam-arduino/issues/6#issuecomment-899100086).**
  - Please add `#include "soc/adc_channel.h"` to `$HOME/Arduino/libraries/Timer-CAM/src/battery.c`
  - The current origin/master fixes this problem, so version 0.0.3 will not have this problem.

## Overview

Published topics:

- `/timer_cam_image/compressed` (`sensor_msgs/CompressedImage`)

  The compressed fisheye image. You can decompress this image by `image_transport/respublish` node. For detail, see [timer_cam.launch](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros/launch/timer_cam.launch)

## Usage

- Follow [README.md](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros)

- Run

  ```bash
  roslaunch m5stack_ros timer_cam.launch
  ```
