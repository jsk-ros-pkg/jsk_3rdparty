and_scale_ros
=============

ROS Interface for A&D scales

## Usage

### EK-i/EW-i series

First, connect a scale to PC with USB-RS232 converter.
Then run the driver:
``` bash
rosrun and_scale_ros ekew_i_driver.py _port:=/dev/ttyUSB0
```
You can get scale value:
``` bash
rostopic echo /ekew_i_driver/output
```