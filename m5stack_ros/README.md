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
    rosdep install --ignore-src --from-paths jsk-ros-pkg/jsk_3rdparty/m5stack_ros -y -r
    cd ~/m5stack_ros_ws
    catkin build m5stack_ros
    ```

2. Setup [Arduino IDE](https://www.arduino.cc/en/software/) for M5Stack

  - Download arduino IDE file from [this page](https://www.arduino.cc/en/software ) and place the unzipped file under home directory like `~/arduino-1.8.16`. I have tested version 1.8.16 and recommend to [download this version](https://downloads.arduino.cc/arduino-1.8.16-linux64.tar.xz).
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
      - Please select version 2.0.0 (> 2.0.1 has bug, 2.0.1 does not work with `TimerCam`).
    - Install M5Stack version 0.3.6 ~ 0.4.0 by [M5Stack](https://github.com/m5stack/M5Stack/tree/0.3.6) (Tools -> Manage Libraries -> M5Stack)
    - Select correct board type (Tools -> Board -> ESP32 Arduino)
      - M5Stack Grey/Base: M5Stack-Core-ESP32
      - M5Stack Fire: M5Stack-Fire
      - Timer Camera F: M5Stack-Timer-CAM

  - Add dependent libraries to arduino library. This script installs libraries that cannot be installed from the Arduino IDE GUI.

    ```bash
    source ~/m5stack_ros_ws/devel/setup.bash
    rosrun m5stack_ros add_libraries.sh $ARDUINO_VERSION
    ```

  - Make rosserial_arduino libraries. If you have already installed [Rosserial_Arduino_Libraries](https://github.com/frankjoshua/rosserial_arduino_lib) from Arduino libraries, please remove it. It has old `ros_lib` directory.

    ```bash
    # If you have already installed Rosserial_Arduino_Libraries, remove
    OLD_ROSSERIAL_DIR=$HOME/Arduino/libraries/Rosserial_Arduino_Library
    if [ -d $OLD_ROSSERIAL_DIR ]; then rm -r $OLD_ROSSERIAL_DIR; fi
    # Make ros_lib basd on your workspaces
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

  - Define macro at the middle of `m5stack_ros/arduino_libraries/m5stack_ros.h` based on connection type between M5Stack and PC

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

  - If you use Wi-Fi, set SSID and password in `m5stack_ros/arduino_libraries/wifi.h`. **DO NOT** upload these information to github.

4.  Start main program

  - Connect M5Stack and the device with cable.

  - Connect M5Stack and PC

    - If you use Bluetooth connection, exec following commands. For detail, see [PR](https://github.com/ros-drivers/rosserial/pull/569).

      ```bash
      # Check if your computer has a bluetooth interface
      hciconfig
      # Bind bluetooth device and rfcomm devce file
      sudo rfcomm bind 1 <Bluetooth MAC Address of M5Stack>
      ```

      - Bluetooth MAC Address is printed on Arduino IDE Serial Monitor when M5Stack is started (Line feed code:only LF, baudrate: 115200bps).

      - If you use the same device repeatedly, I recommend you to add these commands to init daemon like upstart or systemd.

  - source setup.bash

    ```bash
    source ~/m5stack_ros_ws/devel/setup.bash
    ```

  - Launch ROS program. For detail, please see [Devices](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros#devices) section.

    First, it is recommended that you use the `SimplePublisher` firmware. You can verify that the environment has been built correctly without sensors.

    You can check if the microcontroller is operating properly from the Serial Monior. As long as the microcontroller is connected to PC via USB, you can use Serial Monitor regardless of the communication type.

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

  - If you want to test the compilation of codes under the `sketches` directory, execute the following steps
    - Uncomment test section in `m5stack_ros/CMakeLists.txt`
    - Run `catkin run_tests m5stack_ros --no-deps`

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

If you add a new sketch, please follow these rules. Sketch testing depends on these directory configurations.

- Create the directory with the same name as your sketch under `m5stack_ros/sketches` directory
- Locate main ino file under the created directory


## Autostart by Systemd

By using Systemd, the m5stack_ros program can be started automatically.

1. Edit `config/m5stack_ros.service`
    - Write your main program to `ExecStart` section.
    - Be careful with the workspace path.

2. Place the service config file to systemd user direcoty and enable the service

    ```
    rosrun m5stack_ros enable_m5stack_ros_service.sh
    ```

3. Reboot your computer.

4. Check the log of the service. Like `tail` command, you can use `-f` interactive mode or `-n` last lines mode.

   ```
   journalctl --user -u m5stack_ros.service
   sudo -u $USER XDG_RUNTIME_DIR=/run/user/$(id -u $USER) systemctl --user status m5stack_ros.service
   ```

5. If you use bluetooth connection, please run the following additional steps to automatically bind rfcomm devices.

    - Edit `config/rfcomm_bind.service`. Be careful with the workspace path.
    - Edit `config/rfcomm_devices.yaml`, in which Bluetooth MAC address of the M5 device is set.
    - Place the service config file to systemd user direcoty and enable the service
      ```
      rosrun m5stack_ros enable_rfcomm_bind_service.sh
      ```
    - Clone `jsk_robot_startup` to your workspace and catkin build it.
      ```
      cd ~/m5stack_ros_ws/src/jsk-ros-pkg
      git clone https://github.com/jsk-ros-pkg/jsk_robot.git
      catkin build jsk_robot_startup
      ```
    - Reboot your computer
    - Check the log of the service.
      ```
      sudo journalctl -u rfcomm_bind.service
      ```
    - Check the rfcomm binded devices
      ```
      rfcomm
      ```

## Unique device file name by udev rules

When multiple M5 devices are used with USB connections to the same PC, their device files must be distinguished.
With the following steps, different symbolic links are created for each M5 device. You can use [udev file example](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros/config/99-m5stack-ros.rules).

  - Check ATTRS{idVendor} and ATTRS{idProduct} by the following command
    ```
    lsusb
    ```

  - Check ATTRS{serial} by
    ```
    # Be sure that your device is /dev/ttyUSB0
    sudo udevadm info -a -p $(sudo udevadm info -q path -n /dev/ttyUSB0) | grep ATTRS{serial}`
    ```

  - Place 99-m5stack-ros-rules under `/etc/udev/rules.d/`

  - Restart udev
    ```
    sudo udevadm trigger
    ```

  - Reconnect your M5 device and check if symbolic link (/dev/[Device name]) is created.


## Tested environment

### Hardware

- M5Stack Gray (All sketches except EARTH sensor)
- M5StickC (Only EARTH sensor)
- M5StickC Plus (Only EARTH sensor)
- M5Stack Fire (Only SimplePublisher)
- Timer Camera F (Only TimerCam)

### Software

- Ubuntu 18.04
- Arduino IDE 1.8.16
  - esp32 by Espressif Systems 2.0.0
- The arduino libraries on which each firmware depends is described in the respective README.
