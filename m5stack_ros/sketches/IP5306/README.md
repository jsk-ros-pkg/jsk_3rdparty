# IP5306

Publish [IP5306](http://www.injoinic.com/wwwroot/uploads/files/20200221/0405f23c247a34d3990ae100c8b20a27.pdf) battery state

## Overview

Published topics:

- `/battery_level` (`std_msgs/UInt16`)

  Battery level

- `/is_charging` (`std_msgs/Bool`)

  Whether the battery is charged or not

## Usage

- Follow [README.md](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros)

- Run

  ```bash
  roslaunch m5stack_ros m5stack_ros.launch
  ```
