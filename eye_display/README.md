# Eye Display Module

[![PlatformIO Build Workflow](https://github.com/sktometometo/eye-display/actions/workflows/main.yml/badge.svg)](https://github.com/sktometometo/eye-display/actions/workflows/main.yml)
[![Cakin Build and PlatformIO Build Workflow](https://github.com/sktometometo/eye-display/actions/workflows/full.yml/badge.svg)](https://github.com/sktometometo/eye-display/actions/workflows/full.yml)

https://github.com/user-attachments/assets/e2b44bc5-4f85-489f-b862-e851fd4cdf32

Eye Display Module

## Supported devices

1. Round Display Module with M5Stamp C3 (pio env name: `stampc3`) : https://www.switch-science.com/products/8098
2. Round Display Module with M5Stamp S3 (pio env name: `stamps3`) : https://www.switch-science.com/products/8971

## How to use

### Installation

First you have to install ROS and PlatformIO

```bash
pip install platformio
```

And then, you can build and upload the firmware to the device.

It is recommended to put this repo in a catkin workspace.

```bash
mkdir ~/catkin_ws/src
cd ~/catkin_ws
catkin init
cd ~/catkin_ws/src
git clone https://github.com/jsk-ros-pkg/jsk_3rdparty.git
cd eye_display
rosdep install --from-paths . --ignore-src -y -r
catkin build eye_display
source ~/catkin_ws/devel/setup.bash
```
If you do not want to download all jsk_3rdparty files, pleasea check `How to clone only this package` section.

### Simple demo


https://github.com/user-attachments/assets/e2b44bc5-4f85-489f-b862-e851fd4cdf32


You can check basic functionalities with a demo firmware.

```bash
roscd eye_display
pio run -e stampc3-ros
pio run -e stampc3-ros -t uploadfs --upload-port <port to device>
pio run -e stampc3-ros -t upload --upload-port <port to device>
```

Please replace `stampc3` with `stamps3` if you use type 2 device.

After building and uploading the firmware, you can control the device through ROS topic

```bash
roslaunch eye_display demo.launch port:=<port to device> mode_right:=<true or false>
```

Then you can control the device with the demo scripts.

```bash
rosrun eye_display pub_eye_status.py
```

```bash
rosrun eye_display demo_move_eye.py
```


You can also directly control pupil position by publish a message to "/eye_display/look_at" topic.

```bash
rostopic pub -1 /eye_display/look_at geometry_msgs/Point "{x: 40.0, y: -10.0, z: 0.0}"

```

You can control emotion expression with eye by publishing a message to "/eye_display/eye_status" topic.

```bash
rostopic pub -1 /eye_display/eye_status std_msgs/String "data: 'happy'"
```

To get the list of  emotional expression of the eyes, you can use following command.

```bash
$rosparam get eye_display/eye_asset/names
[normal, blink, surprised, sleepy, angry, sad, happy]
```

#### I2C version

If you want to control the device through I2C bus, please use following env.

- `stampc3-i2c`: Stamp C3 device
- `stamps3-i2c`: Stamp S3 device

```bash
roscd eye_display
pio run -e stampc3-i2c
pio run -e stampc3-i2c -t uploadfs --upload-port <port to device>
pio run -e stampc3-i2c -t upload --upload-port <port to device>
```

Then you can control the device with I2C.

```bash
roslaunch eye_display demo.launch use_i2c:=true i2c_device:=<device number> i2c_bus:=<bus number>
```

See `node_scripts/ros_to_i2c.py` for control protocol.

To monitor the serial output in the dual I2C mode. Use the following logger tool.

```bash
./node_scripts/dual_serial_logger.py /dev/ttyACM0 /dev/ttyACM1 115200
```
#### Dual eye mode

You can start two device with `demo_dual.launch`

```bash
roslaunch eye_display demo_dual.launch use_i2c:=false port_right:=/dev/ttyACM0 port_left:=/dev/ttyACM1 baud:=115200 debug:=true
```

You can control dual eye status with demo scripts
```bash
rosrun eye_display pub_eye_status.py --dual --rate 0.3 --names sleepy surprised happy
```

#### extra images

If you need more than the standard images (outline, iris, pupil, reflex, upperlid), use the extra　images.


```
  path_extra1: "/krmt_reflex_shine1.png"
  extra1_default_pos_x: 75
  extra1_default_pos_y: 75
  extra1_default_theta: 0
  extra1_position_x: [  0,  20, 40,  20,   0, -20, -40, -20]
  extra1_position_y: [ 40,  20,  0, -20, -40, -20,   0,  20]
  extra1_rotation_theta: [  0,  40,  80,  40,  0,  -40, -80, -40]
  extra1_zoom: [  1.0, 1.1, 1.2, 1.3, 1.4, 1.2, 1.0, 0.8, 0.7, 0.8, 0.9]
  path_extra2: "/krmt_reflex_heart.png"
  extra2_position_x: [  0,  20,   0, -20]
  extra2_position_y: [  0,  10,   0, -10]
  extra2_rotation_theta: [  0,  -40,  -80,  -40,  0,  40, 80, 40]
  extra2_zoom: [1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 1.5, 1.0, 0.5]
  extra2_default_pos_x: 75
  extra2_default_pos_y: 75
  extra2_default_theta: 0
```

#### Advanced control

You can control the eye status in more detail through the `eye_status` topics.
Use the following low-level commands.
`<emotion>` is defined in the names list in the configuration YAML file (e.g., `names: [normal, happy, blink]`).
`<type>` can be one of `iris`, `pupil`, `reflex`, `upperlid`, `extra1`, `extra2`.

```
eye_asset_image_path: <emotion>: <type>: <file name>
eye_asset_default_pos_x: <emotion>: <type>: <value>
eye_asset_default_pos_y: <emotion>: <type>: <value>
eye_asset_default_theta: <emotion>: <type>: <value in degree>
eye_asset_default_zoom: <emotion>: <type>: <value>
eye_asset_position_x: <emotion>: <type>: <comma separated values>
eye_asset_position_y: <emotion>: <type>: <comma separated values>
eye_asset_rotation_theta: <emotion>: <type>: <comma separated values>
eye_asset_zoom: <emotion>: <type>: <comma separated values>
```

### Description of direction

![eye_display_direction](./doc/eye_display_direction.svg)

### How to update image

![eye_layer_structure](./doc/eye_structure.svg)

## For Developers

### How to update msg

Message headers in [`lib/ros_lib`](./lib/ros_lib/) directory are automatically generated with `make_libraries.py` script in `rosserial_arduino` package.

And this repo provide easy way to update `ros_lib` as [update_ros_lib.sh](./scripts/update_ros_lib.sh).

So if you want to update message definition in [`msg`](./msg/) directory, please run the following command.

```bash
catkin build eye_display
source <path/to/catkin_ws>/devel/setup.bash
rosrun eye_display update_ros_lib.sh
```

### How to clone only this package

This feature requires git 2.27+, so if you use Ubuntu<=20.04, please install latest version of git https://git-scm.com/downloads/linux

```
git clone --filter=blob:none --sparse https://github.com/jsk-ros-pkg/jsk_3rdparty.git
cd jsk_3rdparty
git sparse-checkout set eye_display
git checkout eye_display
```
