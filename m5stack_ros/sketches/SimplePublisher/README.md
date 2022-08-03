# SimplePubisher

ROS publisher sample using m5stack_ros

## Overview

Communication:

- UnitV <-(UART)-> M5Stack <-(Bluetooth/Wi-Fi/USB)-> PC
- Connect M5Stack and UnitV with Grove cable

Published topics:

- `/unitv_image` (`sensor_msgs/Image`)

  UnitV Image compressed to jpg

## Usage

```bash

roslaunch m5stack_ros m5stack_ros.launch
```
