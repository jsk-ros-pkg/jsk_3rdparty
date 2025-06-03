and_scale_ros
=============

ROS Interface for [A&D scales](https://www.aandd.co.jp/adhome/products/index_weighing.html#01)

## Usage

### [EK-i](https://www.aandd.co.jp/adhome/products/balance/ek-i.html)/[EW-i](https://www.aandd.co.jp/adhome/products/balance/ew-i.html) series

First, connect a scale to PC with USB-RS232 converter.
Then run the driver:
``` bash
rosrun and_scale_ros ekew_i_driver.py _port:=/dev/ttyUSB0
```
or
``` bash
roslaunch and_scale_ros ekew_i_driver.launch
```
You can get scale value:
``` bash
rostopic echo /ekew_i_driver/output
```
