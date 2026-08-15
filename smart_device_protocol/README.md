<p align="center">
  <img src="./docs/sdp_logo_v1.png" width="300px" />
</p>

<div align="center">

[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![Black formatting](https://github.com/sktometometo/smart_device_protocol/actions/workflows/python_black.yml/badge.svg)](https://github.com/sktometometo/smart_device_protocol/actions/workflows/python_black.yml)
[![ROS build workflow](https://github.com/sktometometo/smart_device_protocol/actions/workflows/catkin_build.yml/badge.svg)](https://github.com/sktometometo/smart_device_protocol/actions/workflows/catkin_build.yml)
[![PlatformIO Build Workflow](https://github.com/sktometometo/smart_device_protocol/actions/workflows/platformio.yml/badge.svg)](https://github.com/sktometometo/smart_device_protocol/actions/workflows/platformio.yml)

</div>

# smart_device_protocol

The Smart Device Protocol (SDP) Repository.

## What is this?

Smart Device Protocol (SDP) is a protocol to communicate between smart devices, wearable devices, and robots.
SDP is based on [ESP-NOW](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html) protocol, which is a protocol to communicate between ESP32 devices.

This package provides firmwares and ROS nodes to communicate with ESP32 devices via Smart Device Protocol.

For more details of Smart Device Protocol, please see [Smart Device Protocol Document](./docs/sdp.md)

## How to use

### 0. Prerequisites

This code is developed under environment below. Other environment is not tested.

- Ubuntu 20.04
- ROS Noetic
- Python 3.8

And you have to install `platformio` pip package to your environment.
  
  ```bash
  pip3 install platformio
  ```

### 1. Build ROS package

First, clone this repository to your catkin workspace and build it.

  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/sktometometo/smart_device_protocol.git
  cd ..
  catkin build
  ```

### 2. Make Smart Device Protocol interface device

Second, you have to make an ESP32 device which is connected to your PC via USB.
This device will be the interface to Smart Device Protocol communication of your PC.
Basically, you can use any ESP32 device, but this package is tested with M5Stack-Fire, M5Stack-Core2, and M5Atom-S3.

We will use [smart_device_protocol_interface](./sketchbooks/smart_device_protocol_interface) project.
This code is developed with [PlatformIO](https://platformio.org/). So you can build and burn firmware with it.

You can execute `pio` command.

  ```bash
  ~ $ pio
  Usage: pio [OPTIONS] COMMAND [ARGS]...

  Options:
    --version          Show the version and exit.
    -c, --caller TEXT  Caller ID (service)
    --no-ansi          Do not print ANSI control characters
    -h, --help         Show this message and exit.

  Commands:
    access    Manage resource access
    account   Manage PlatformIO account
    boards    Board Explorer
    check     Static Code Analysis
    ci        Continuous Integration
    debug     Unified Debugger
    device    Device manager & Serial/Socket monitor
    home      GUI to manage PlatformIO
    org       Manage organizations
    pkg       Unified Package Manager
    project   Project Manager
    remote    Remote Development
    run       Run project targets (build, upload, clean, etc.)
    settings  Manage system settings
    system    Miscellaneous system commands
    team      Manage organization teams
    test      Unit Testing
    upgrade   Upgrade PlatformIO Core to the latest version
  ```

Then, connect M5Stack-Core2 to your PC. You can check which port is connected to it by

  ```bash
  ~ $ pio device list
  /dev/ttyACM0
  ------------
  Hardware ID: USB VID:PID=1A86:55D4 SER=54BB013663 LOCATION=7-1:1.0
  Description: USB Single Serial
  ```

So let's build firmware and burn it to M5Stack-Core2

  ```bash
  roscd smart_device_protocol/sketchbooks/smart_device_protocol_interface/
  pio run -e m5stack-core2 --target upload --upload-port /dev/ttyACM0
  ```

### 3. Make an example of Smart Device Protocol device

In this tutorial, you will see your PC can communicate with ESP32 device via Smart Device Protocol. So you have to make another ESP32 Smart Device Protocol device.
We will use [sdp_example](./sketchbooks/sdp_example/).

Connect M5Stack-Fire to your PC and burn firmware to it.

  ```bash
  roscd smart_device_protocol/sketchbooks/sdp_example/
  pio run -e m5stack-fire --target upload --upload-port /dev/ttyACM0
  ```
  
### 4. Run Smart Device Protocol Interface Device

After step 2, you can run Smart Device Protocol interface node.

  ```bash
  roslaunch smart_device_protocol demo.launch port:=/dev/ttyACM0
  ```

with this, you can see topics below.

  ```bash
  $ rostopic list
  /diagnostics
  /smart_device_protocol/recv
  /smart_device_protocol/send
  /rosout
  /rosout_agg
  ```

You can send a ESP-NOW (which is the bottom of Smart Device Protocol) packet directly by sending ROS a message to `/smart_device_protocol/send` topic.

  ```bash
  rostopic pub -1 /smart_device_protocol/send smart_device_protocol/Packet "mac_address: [255, 255, 255, 255, 255, 255]
  data: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]"
  ```

### 5. Run a ROS Node with Smart Device Protocol Interface

Now you can communicate with smart devices, wearable devices, and robots via Smart Device Protocol.
You can run an example of Smart Device Protocol Interface node.

  ```bash
  rosrun smart_device_protocol sdp_v2_packet_printer.py
  ```

With this command, you can see Smart Device Protocol packets from other devices.

  ```bash
  TODO
  ```

You can also send Smart Device Protocol packets to other devices.

  ```bash
  TODO
  ```

## API for Smart Device Protocol

### Python API (for ROS Node)

You can use Smart Device Protocol API in Python for ROS Node. Please see [this document](./docs/python.md) for more details.

### C++ API (for Arduino)

You can use Smart Device Protocol API in C++ for Arduino. Please see (this document)[./arduino_lib/README.md] for more details.

## Examples of Smart Device Protocol Interface Device

There are some examples of Smart Device Protocol Interface Device. For more details, please see [this directory](./sketchbooks/).

## Notices

### Update of ros_lib for Arduino

If you update ros_lib for Arduino, you have to update `ros_lib` directory in [this directory](./ros_lib/).

  ```bash
  cd ~/catkin_ws/src/smart_device_protocol/ros_lib
  rm -rf ros_lib
  rosrun rosserial_arduino make_libraries.py .
  ```