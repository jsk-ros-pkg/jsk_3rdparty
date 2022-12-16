# TimerCam

Publish [TimerCam](https://docs.m5stack.com/en/unit/timercam_f) fisheye image

## Dependencies

Install the following packages from Arduino libraries (Tools -> Manage Libraries)
- [Timer-CAM](https://github.com/m5stack/TimerCam-arduino/tree/0.0.2) version 0.0.2
  - NOTE that **compile will fail without [this change](https://github.com/m5stack/TimerCam-arduino/issues/6#issuecomment-899100086).**
  - Please add `#include "soc/adc_channel.h"` to `$HOME/Arduino/libraries/Timer-CAM/src/battery.c`
    - The current origin/master fixes the above problem, so version 0.0.3 will not have this problem.
  - Please add `conf.clk_flags = 0;` to `i2c_init()` in `$HOME/Arduino/libraries/Timer-CAM/src/bmm8563.h`. For detail, see https://msr-r.net/timer-camera-lib/

This sketch works with [esp32 by Espressif Systems](https://github.com/espressif/arduino-esp32/tree/2.0.0) == 2.0.0 and does not works with >= 2.0.1. This is because [this directory](https://github.com/espressif/arduino-esp32/tree/2.0.0/tools/sdk/esp32/include/esp-face) is removed in the newer versions.

## Overview

Published topics:

- `/timer_cam_image/compressed` (`sensor_msgs/CompressedImage`)

  The compressed fisheye image. You can decompress this image by `image_transport/respublish` node. For detail, see [timer_cam.launch](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros/launch/timer_cam.launch)

- `/battery_level` (`std_msgs/UInt16`)

  Battery level [mV] of the TimerCam. If TimerCam is fully charged, the value is about 4200. You can use `BASE_VOLATAGE` 3600 (typo in the origin!) as threshold.

## Usage

- Follow [README.md](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros)

- Run

  ```bash
  roslaunch m5stack_ros timer_cam.launch
  ```

- If you want to tune the camera parameters, use [web_cam](https://github.com/m5stack/TimerCam-arduino/blob/master/examples/web_cam/web_cam.ino) sample.
  - Burn the firmware
  - Open web browser and go to `http://(camera IP address)`. For detail, open Serial Monitor and read the log output.

- If the focal length is not correct, turn the black part around the lens. See also [m5-docs](https://docs.m5stack.com/en/unit/timercam).
