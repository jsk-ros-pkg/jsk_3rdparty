# EARTH

Publish [Grove - Gas Sensor V2(Multichannel)](https://wiki.seeedstudio.com/Grove-Multichannel-Gas-Sensor-V2/)

## Overview

Published topics:

- `/gas_v2_102b` (`std_msgs/UInt16`)

  Nitrogen dioxide (NO2)

- `/gas_v2_302b` (`std_msgs/UInt16`)

  Ethyl alcohol(C2H5CH)

- `/gas_v2_502b` (`std_msgs/UInt16`)

  Volatile Organic Compounds (VOC)

- `/gas_v2_702b` (`std_msgs/UInt16`)

  Carbon monoxide (CO)

## Usage

- Follow [README.md](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros)

- Run

  ```bash
  roslaunch m5stack_ros m5stack_ros.launch
  ```
