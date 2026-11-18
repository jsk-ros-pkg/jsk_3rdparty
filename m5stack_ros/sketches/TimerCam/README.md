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

## Stereo

### Calibration

You can use [3D CAD data](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros/3d_models/TimerCam/Timer-cam-stereo-3cm.STL) to create stereo camera by fixing 2 TimerCam. The baseline is 3cm.

Calibrate TimerCam stereo

```
# Launch TimerCam images
roslaunch m5stack_ros timercam_stereo.launch
# Calibration
rosrun camera_calibration cameracalibrator.py --approximate 0.5 --size 7x10 --square 0.025 right:=/right/image_raw left:=/left/image_raw
```

Place stereo config files

```
cd /tmp
tar xvzf calibrationdata.tar.gz
cp /tmp/calibrationdata/left.yaml $(rospack find m5stack_ros)/config/timercam_left_3cm.yaml
cp /tmp/calibrationdata/right.yaml $(rospack find m5stack_ros)/config/timercam_right_3cm.yaml
```

### Publish pointcloud

```
roslaunch m5stack_ros timercam_stereo.launch
rviz -d $(rospack find m5stack_ros)/config/timercam_stereo.rviz
```

### Tips

- StereoSGBM is the recommended stereo algorithm over StereoBM.
- Since two TimerCams are used, it is better to distinguish them by udev. `ATTRS{idVendor}`, `ATTRS{idProduct}` and `ATTRS{serial}` can be specified to distinguish them.
  ```
    $ udevadm info -q property -n /dev/ttyUSB0 | grep -E "ID_SERIAL_SHORT=|ID_VENDOR_ID=|ID_MODEL_ID="
  ```
- To confirm the success or failure of stereo calibration, check the value of the projection matrix. The baseline value can be calculated by dividing the elements of this matrix. (232.17283 / -7.303 = -31.79). The baseline length in the CAD model is 30 mm, and if the calculated value is close to this value, the calibration is successful.

  ```
  $ cat $(rospack find m5stack_ros)/config/timercam_right_3cm.yaml

  ...

  projection_matrix:
  rows: 3
  cols: 4
  data: [232.17283,   0.     , 140.26832,  -7.303  ,
           0.     , 232.17283, 117.94981,   0.     ,
           0.     ,   0.     ,   1.     ,   0.     ]

  ```

- If you want to perform stereo matching with higher precision, you can use deep learning-based methods.

- For faster stereo matching, you can reduce the number of camera pixels by [TimerCam.ino](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros/sketches/TimerCam/TimerCam.ino).
  ```
  s->set_framesize(s, FRAMESIZE_QVGA);   // 320x240
  ```

- (Unverified) When a port other than the default 11411 port was specified in the `rosrun rosserial_python serial_node.py tcp` command, communication sometimes became unstable. In that case, communication with TimerCam may be better done via USB connection.
