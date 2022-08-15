# unitv

Publish [UnitV](https://shop.m5stack.com/products/unitv-ai-camera) recognition result as ROS topic via M5Stack

## Dependencies

Install the necessary packages with `apt`
```
# To decompress rgb image by image_transport/republish node
sudo apt install ros-$ROS_DISTRO-image-transport-plugins
```

## Overview

Communication:

- UnitV <-(UART)-> M5Stack <-(Bluetooth/Wi-Fi/USB)-> PC
- Connect M5Stack and UnitV with Grove cable

Published topics:

- `/unitv_image` (`sensor_msgs/Image`)

  UnitV Image compressed to jpg

- `/unitv_image/rects` (`jsk_recognition_msgs/RectArray`)

  Object position rectangle

- `/unitv_image/class` (`jsk_recognition_msgs/ClassificationResult`)

  Object class

- `/draw_rects/output` (`sensor_msgs/Image`)

  UnitV image with recognition result

Visualization of `/draw_rects/output`:

![unitv demo](https://user-images.githubusercontent.com/19769486/132894050-84d7a79d-da3a-4057-8487-025188f65515.gif)

## Setup

In this section, you can use yolov2 recognition.

1. Prepare [M5Stack](https://shop.m5stack.com/collections/m5-controllers/products/basic-core-iot-development-kit) and [UnitV](https://shop.m5stack.com/products/unitv-ai-camera?variant=34757279809700)

2. Prepare Arduino IDE and ROS workspace based on [README.md](https://github.com/jsk-ros-pkg/jsk_3rdparty/blob/master/m5stack_ros/README.md)

3. Install [kflash](https://github.com/sipeed/kflash_gui/releases/download/v1.5.3/kflash_gui_v1.5.3_linux.tar.xz) and [MaixPy IDE](https://dl.sipeed.com/shareURL/MAIX/MaixPy/ide/_)

    - About kflash and MaixPy IDE, see [unitv quick start](https://docs.m5stack.com/en/quick_start/unitv/unitv_quick_start_maixpy)

4. Burn firmwares into UnitV

    - Make sure that UnitV is connected to your computer via USB Type-C
    - Burn [yolov2.kfpkg](https://github.com/jsk-ros-pkg/jsk_3rdparty/blob/master/m5stack_ros/unitv/data/yolov2.kfpkg) with kflash
    - Burn [yolov2.py](https://github.com/jsk-ros-pkg/jsk_3rdparty/blob/master/m5stack_ros/unitv/firmware/yolov2/yolov2.py)  as boot.py with MaixPy (Tools -> Save open script to board(boot.py))

5. Burn [yolov2.ino](https://github.com/jsk-ros-pkg/jsk_3rdparty/blob/master/m5stack_ros/unitv/firmware/yolov2/yolov2/yolov2.ino) into M5Stack with Arduino IDE.

    - There are 2 connection types between M5Stack and computer: Bluetooth and Wi-Fi

    - Do not forget to setup connection according to your connection type. (e.g. edit #define line or Wi-Fi SSID)

    - Currently, USB connection is deprecated. This is because USB connection cannot use baud rate >= 115200. Baud rate 57600 is so slow that UnitV image cannot be transferred.

## Run

1. Setup Connection.

    - Connect M5Stack and UnitV with Grove cable.

    - If you use Bluetooth connection, exec following commands. For detail, see [this pull request](https://github.com/ros-drivers/rosserial/pull/569).

      ```bash
      sudo rfcomm bind 1 <Bluetooth MAC Address of M5Stack>
      sudo stty -F /dev/rfcomm1 115200 cs8
      ```

    - Bluetooth MAC Address is printed on Arduino IDE Serial Monitor when M5Stack is started.

2. Launch program: Start UniV recognition and visualize it.

    First, source setup.bash

    ```bash
    source ~/catkin_ws/devel/setup.bash
    ```

    - For Bluetooth,

      ```bash
      roslaunch m5stack_ros yolov2.launch baud:=115200 port:=/dev/rfcomm1
      ```

    - For Wi-Fi,

      ```bash
      roslaunch m5stack_ros yolov2.launch baud:=115200 port:=tcp
      ```

## Trouble shooting

If visualization stops or does not start, follow the below instructions:

- Make sure that M5Stack and UnitV are connected with Grove cable
- Unplug the USB cable from M5Stack
- Double-click the main button to sleep M5Stack
- Plug the USB cable to M5Stack or click the main button to boot the M5Stack

Cannot coexist with I2C:

- In this application, Grove connector is used for UART communication.
- If you connect UnitV to M5Stack Grove connector, you cannot use UART and I2C devices simultaneously. In that case, try `enableI2C` and `disableI2C` functions defined in [yolov2_with_battery.ino](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros/sketches/UnitV/yolov2/yolov2_with_battery/yolov2_with_battery.ino).
- The sample program which uses UART UnitV and I2C battery management module is under [yolov2_with_battery](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros/sketches/UnitV/yolov2/yolov2_with_battery)
  - When acquiring battery information, the connection to the UnitV temporarily stops.
  - Therefore, there are times when the UnitV information is temporarily unavailable

If you want to use other recognition model, check following:

- Find or create `.kfpkg` file, which includes UnitV bin file and recognition model.
  - Find the models from [sipeed models](https://dl.sipeed.com/shareURL/MAIX/MaixPy/model) and [MaixHub](https://www.maixhub.com/)
  - Create the model at [MaixHub Model Training](https://www.maixhub.com/ModelTraining)
  - `.kfpkg` file is the same as zip file, so you can use the `.kfpkg` file just like zip file.
- Write script for recognition.
  - Sample MicroPython code for UnitV are in [sipeed GitHub](https://github.com/sipeed/MaixPy_scripts/tree/master/machine_vision)
  - You also need to edit launch file for visualization in ROS. Useful visualization nodes are [draw_classification_result.py](https://jsk-docs.readthedocs.io/projects/jsk_recognition/en/latest/jsk_perception/nodes/draw_classification_result.html), [draw_rects.py](https://jsk-docs.readthedocs.io/projects/jsk_recognition/en/latest/jsk_perception/nodes/draw_rects.html) and [label_image_decomposer.py](https://jsk-docs.readthedocs.io/projects/jsk_recognition/en/latest/jsk_perception/nodes/label_image_decomposer.html)
