# m5stack_ros

Connect the devices to ROS via [M5Stack](https://m5stack.com/) and [rosserial](https://github.com/ros-drivers/rosserial).

## Usage

1. Create ROS workspace (Skip if you already have jsk_3rdparty)

    ```bash
    mkdir ~/m5stack_ros_ws/src -p
    cd ~/m5stack_ros_ws/src
    git clone https://github.com/jsk-ros-pkg/jsk_3rdparty.git jsk-ros-pkg/jsk_3rdparty
    ```

2. Download dependencies
    ```
    cd ~/m5stack_ros_ws/src
    wstool init
    wstool merge ./jsk-ros-pkg/jsk_3rdparty/m5stack_ros/m5stack_ros.rosinstall
    wstool up
    rosdep install --ignore-src --from-path . -y -r
    cd ~/m5stack_ros_ws
    catkin build m5stack_ros
    ```

2. Setup [Arduino IDE](https://www.arduino.cc/en/software/) for M5Stack

  - Download arduino IDE file from [this page](https://www.arduino.cc/en/software ) and place the unzipped file under home directory like `~/arduino-1.8.16`.
    ```bash
    ARDUINO_VERSION=1.8.16 # Set your Arduino version to environment variable
    cd ~
    tar xvf ~/Downloads/arduino-$ARDUINO_VERSION-linux64.tar.xz
    mv ~/Downloads/arduino-$ARDUINO_VERSION-linux64.tar.xz ~/
    ```

  - Setup linux system for Arduino IDE

    ```bash
    ~/arduino-$ARDUINO_VERSION/arduino-linux-setup.sh $USER
    ```

  - Add esp32 to board manager
    - Open arduino IDE. `~/arduino-$ARDUINO_VERSION/arduino`
    - Add following Boards Manager URLs (File -> Preferences -> Additional Boards Manager URLs -> Click icon)
      - `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
      - `https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/arduino/package_m5stack_index.json`
    - Install esp32 by Espressif Systems (Tools -> Board -> Boards Manager -> esp32)
      - Please select correct version (recommed version is 2.0.0). Check [software version](https://github.com/708yamaguchi/jsk_3rdparty/tree/m5stack-ros-/m5stack_ros#software) for more detail
    - Install M5Stack by M5Stack (Tools -> Manage Libraries -> M5Stack)
    - Select correct board type (Tools -> Board -> ESP32 Arduino)
      - M5Stack Grey/Base: M5Stack-Core-ESP32
      - M5Stack Fire: M5Stack-Fire
      - Timer Camera F: M5Stack-Timer-CAM

  - Create Symlink from m5stack_ros library to arduino library

    ```bash
    source ~/m5stack_ros_ws/devel/setup.bash
    rosrun m5stack_ros symlink_libraries.sh $ARDUINO_VERSION
    ```

  - Make rosserial_arduino libraries

    ```bash
    source ~/m5stack_ros_ws/devel/setup.bash
    cd ~/arduino-$ARDUINO_VERSION/libraries
    rm -rf ros_lib
    rosrun rosserial_arduino make_libraries.py .
    ```

    See also [rosserial_arduino](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)

3. Burn firmware for each device. Available devices are listed in [Devices](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros#devices) section.

  - Launch Arduino IDE

    ```bash
    ~/arduino-$ARDUINO_VERSION/arduino
    ```

  - Select USB port of M5Stack (Tools -> Port -> /dev/ttyUSB*)

  - Define macro at the beginning of `arduino_libraries/m5stack_ros.h` based on connection type between M5Stack and PC

    - Bluetooth

      ```C
      #define ROSSERIAL_ARDUINO_BLUETOOTH
      ```

    - Wi-Fi

      ```C
      #define ROSSERIAL_ARDUINO_TCP
      ```

    - USB

      ```C
      // define nothing
      ```

  - If you use Wi-Fi, set SSID and password in `arduino_libraries/wifi.h`. **DO NOT** upload these information to github.

4.  Start main program

  - Connect M5Stack and the device with cable.

  - Connect M5Stack and PC

    - If you use Bluetooth connection, exec following commands. For detail, see [PR](https://github.com/ros-drivers/rosserial/pull/569).

      ```bash
      sudo rfcomm bind 1 <Bluetooth MAC Address of M5Stack>
      sudo stty -F /dev/rfcomm1 115200 cs8
      ```

      - Bluetooth MAC Address is printed on Arduino IDE Serial Monitor when M5Stack is started.

      - If you use the same device repeatedly, I recommend you to add these commands to init daemon like upstart or systemd.

  - source setup.bash

    ```bash
    source ~/m5stack_ros_ws/devel/setup.bash
    ```

  - Launch ROS program. For detail, please see [Devices](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros#devices) section.

    - For USB, (USB serial seems to be max 57600 baud rate)

      ```bash
      roslaunch m5stack_ros [device].launch baud:=57600 port:=(USB DEVICE like /dev/ttyUSB0)
      ```

    - For Bluetooth,

      ```bash
      roslaunch m5stack_ros [device].launch baud:=115200 port:=/dev/rfcomm1
      ```

    - For Wi-Fi,

      ```bash
      roslaunch m5stack_ros [device].launch baud:=115200 port:=tcp
      ```

## Devices

With this package, you can use following devices. To use each device, please see README.md.

- [UnitV](https://shop.m5stack.com/products/unitv-ai-camera)

    - yolov2 object detection
    - [README](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros/sketches/UnitV)

- [MLX90640](https://www.sparkfun.com/products/14843)

    - Thermography camera
    - [README](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros/sketches/MLX90640)

- [VL53L0X](https://www.adafruit.com/product/3317)

    - ToF proximity sensor
    - [README](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros/sketches/VL53l0X)

- [VCNL4040](https://www.sparkfun.com/products/15177)

    - IR intensity proximity sensor
    - [README](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros/sketches/VCNL4040)

- [IP5306](http://www.injoinic.com/wwwroot/uploads/files/20200221/0405f23c247a34d3990ae100c8b20a27.pdf)

    - M5Stack's internal battery module
    - [README](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros/sketches/IP5306)

- [MPU9250](https://invensense.tdk.com/download-pdf/mpu-9250-datasheet/)

    - M5Stack Gray's internal 9-axis IMU
    - [README](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros/sketches/MPU9250)

- [TVOC_SGP30](https://shop.m5stack.com/products/tvoc-eco2-gas-unit-sgp30)

    - TVOC/eCO2 Gas Sensor Unit
    - [README](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros/sketches/TVOC_SGP30)
- [PDM_SPM1423](https://shop.m5stack.com/products/pdm-microphone-unit-spm1423)

    - Digital MEMS silicon-based microphone based on PDM (Pulse Density Modulation) signal.
    - [README](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros/sketches/PDM_SPM1423)

- [JOYSTICK](https://docs.m5stack.com/en/unit/joystick)

    - Joystick control input unit, supports three-axis control signal input (X/Y-axis offset analog input, Z-axis key digital input).
    - [README](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros/sketches/JOYSTICK)

- [EARTH](https://shop.m5stack.com/products/earth-sensor-unit)

    - Soil Moisture Sensor for measuring the moisture in soil and similar materials.
    - [README](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros/sketches/EARTH)

- [TimerCam](https://docs.m5stack.com/en/unit/timercam_f)

    - The Timer Camera F is a fisheye camera module based on ESP32-D0WDQ6-V3 with 8M PSRAM and 4M Flash on board.
    - [README](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros/sketches/TimerCam)

## Tested environment

### Hardware

- M5Stack Gray

### Software

- Ubuntu 18.04
- Arduino 1.8.16
  - esp32 (Boards Manager): 2.0.0 ~ 2.0.1 (> 2.0.1 has bug)
  - M5Stack (Library Manager): 0.3.6 ~ 0.4.0
