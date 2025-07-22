and_scale_ros
=============

ROS Interface for [A&D scales](https://www.aandd.co.jp/adhome/products/index_weighing.html#01)

## Usage

### Scales supporting A&D standard format

For instance:
- [EK-i](https://www.aandd.co.jp/adhome/products/balance/ek-i.html)/[EW-i](https://www.aandd.co.jp/adhome/products/balance/ew-i.html) series
- [FZ-i](https://www.aandd.co.jp/products/weighing/balance/bal-top-loading/fz-i/)/[FX-i](https://www.aandd.co.jp/products/weighing/balance/bal-top-loading/fx-i/)/[FZ-iWP](https://www.aandd.co.jp/products/weighing/balance/bal-top-loading/fz-iwp/)/[FX-iWP](https://www.aandd.co.jp/products/weighing/balance/bal-top-loading/fx-iwp/) series

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
