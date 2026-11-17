# switchbot_ros

Controls [SwitchBot](https://www.switch-bot.com/) devices from ROS through the
[SwitchBot API](https://github.com/OpenWonderLabs/SwitchBotAPI). `switchbot_ros_server.py` publishes the list of
your devices and provides an action to send commands to them (press a Bot, turn on a light, an infrared remote, ...);
`switchbot_status_publisher.py` publishes the status of one device (a meter, a plug, a bot, ...) periodically.
Client libraries for rospy/rclpy and roseus are included.

## System Component

![Screenshot from 2021-07-17 12-43-06](https://user-images.githubusercontent.com/27789460/126024618-159f57e6-d17e-448f-a606-425b33bc1fff.png)

JSK members can edit the original file [here](https://docs.google.com/presentation/d/18LIq5SMnkFJGJsdDV85V6G5Vyt8o6_0rvvhKwSjwWmk/edit?usp=sharing).

## Requirements

1. Install the SwitchBot app on your phone (iPhone, Android) and create your SwitchBot account.
2. Set up the SwitchBot devices with the app. Give each device a name (the `device_name` of the action) and enable
   the cloud service. Devices which talk Bluetooth only (e.g. a Bot) need a SwitchBot Hub to be controlled from the
   cloud.

   <img src="https://user-images.githubusercontent.com/27789460/121886201-1b0f5700-cd50-11eb-80d2-42cd98554905.PNG" width=150> <img src="https://user-images.githubusercontent.com/27789460/121886458-6164b600-cd50-11eb-81fd-3b9fef35518d.PNG" width=150> <img src="https://user-images.githubusercontent.com/27789460/121886467-6295e300-cd50-11eb-8c33-7fcbcbafb9ac.PNG" width=150> <img src="https://user-images.githubusercontent.com/27789460/121886470-63c71000-cd50-11eb-93a1-274ed98a96e6.PNG" width=150> <img src="https://user-images.githubusercontent.com/27789460/121886477-64f83d00-cd50-11eb-9d51-437b4d18fa8d.PNG" width=150>

3. Get your token and secret: in the app, open Profile > Preferences (Settings), tap the app version 10 times;
   the developer options show the token and the secret key. With a secret, the nodes use the SwitchBot API v1.1;
   without it (empty), the API v1.0. The token and the secret can be given as they are or as the path of a file of
   them.

For JSK members, please see [this slide](https://docs.google.com/presentation/d/11UkuxVT4u_LcAYJQnPvt4mzxos9Qvflth5hgZmtqmyk/edit?usp=sharing)
for the account, the token and the SwitchBots installed in our lab.

The nodes need the internet; the Python package `requests` (`python3-requests`) is used.

## Usage

### ROS 2

```bash
colcon build --packages-up-to switchbot_ros
source install/setup.bash
ros2 launch switchbot_ros switchbot.launch.xml token:=YOUR_TOKEN secret:=YOUR_SECRET
# with the status publisher of one device
ros2 launch switchbot_ros switchbot.launch.xml token:=YOUR_TOKEN secret:=YOUR_SECRET \
  pub_status:=true pub_device_name:=YOUR_DEVICE_NAME
```

See the devices and send a command:

```bash
ros2 topic echo --once /switchbot_ros/devices
ros2 action send_goal --feedback /switchbot_ros/switch switchbot_ros/action/SwitchBotCommand \
  "{device_name: '/myroom/light', command: 'turnOn'}"
ros2 topic echo /switchbot_status_publisher/YOUR_DEVICE_NAME
```

Without the launch file:

```bash
ros2 run switchbot_ros switchbot_ros_server.py --ros-args -r __node:=switchbot_ros \
  -p token:=YOUR_TOKEN -p secret:=YOUR_SECRET
```

### ROS 1

```bash
roslaunch switchbot_ros switchbot.launch token:=YOUR_TOKEN secret:=YOUR_SECRET
```

```bash
rostopic echo -n 1 /switchbot_ros/devices
rostopic pub -1 /switchbot_ros/switch/goal switchbot_ros/SwitchBotCommandActionGoal \
  "{goal: {device_name: '/myroom/light', command: 'turnOn', parameter: '', command_type: ''}}"
rostopic echo /switchbot_ros/switch/result
```

### Launch arguments

`switchbot.launch` (ROS 1) and `switchbot.launch.xml` (ROS 2) have the same arguments:

| Argument | Default | Meaning |
|---|---|---|
| `token` | (required) | the token, or the path of a file of it |
| `secret` | `''` | the secret key, or the path of a file of it; empty uses the API v1.0 |
| `respawn` | `true` | respawn the nodes |
| `pub_status` | `false` | also start `switchbot_status_publisher` |
| `pub_status_rate` | `0.1` | the rate [Hz] of the status publisher |
| `pub_device_name` | `bot74a` | the device of the status publisher |

## Interface

### switchbot_ros_server.py

The node is named `switchbot_ros` by the launch files (`switchbot` when run without them).

Published topics:

- `~devices` (`switchbot_ros/DeviceArray`, latched): the `name` and `type` of your devices and infrared remotes,
  published once the API client is ready.

Actions:

- `~switch` (`switchbot_ros/SwitchBotCommand`): sends a command to a device.
  - goal: `device_name` (the name set in the app), `command` (e.g. `turnOn`, `turnOff`, `press`), `parameter`
    (empty means `default`), `command_type` (empty means `command`; `customize` for a user-defined button of an
    infrared remote). See [the command set](https://github.com/OpenWonderLabs/SwitchBotAPI#send-device-control-commands).
  - feedback: `status`, the message of the API or the error.
  - result: `done`, `true` if the command was accepted. The goal always ends as succeeded; check `done`.

Parameters:

- `~token` (string, required): the token, or the path of a file of it.
- `~secret` (string, default: none): the secret key, or the path of a file of it. Empty or none uses the API v1.0.

### switchbot_status_publisher.py

Published topics:

- `~<device_name>` (`-` in the name replaced by `_`, latched): the status of the device, of the type of the
  device:

  | Device type | Message |
  |---|---|
  | `Meter`, `MeterPlus`, `WoIOSensor` | `switchbot_ros/Meter` |
  | `MeterPro(CO2)` | `switchbot_ros/MeterProCO2` |
  | `Hub 2` | `switchbot_ros/Hub2` |
  | `Plug Mini (JP)`, `Plug Mini (US)` | `switchbot_ros/PlugMini` |
  | `Bot` | `switchbot_ros/Bot` |
  | `Strip Light` | `switchbot_ros/StripLight` |

  Other types (and infrared remotes) are not supported.

Parameters:

- `~token` (string, required), `~secret` (string, default: none): as above.
- `~device_name` (string, required): the device to publish.
- `~rate` (float, default: `0.1`): the rate [Hz] of the requests. The API allows 10000 requests a day.

### Client libraries

#### Python (rospy / rclpy)

```python
# ROS 1
import rospy
from switchbot_ros.switchbot_ros_client import SwitchBotROSClient

rospy.init_node('switchbot_client')
client = SwitchBotROSClient()  # actionname='switchbot_ros/switch', topicname='switchbot_ros/devices'
print(client.get_devices())
client.control_device('/eng2/7f/73b2_bot_kitchen', 'turnOn', wait=True)
```

On ROS 2 pass the node, and spin it in another thread since the calls block; see `scripts/control_switchbot.py`
(`ros2 run switchbot_ros control_switchbot.py`, which turns on the devices `pendant-light` and `bot74a` of JSK):

```python
import threading
from switchbot_ros.switchbot_ros_client import SwitchBotROSClient, ros_compat

node = ros_compat.init_node('switchbot_client')
threading.Thread(target=ros_compat.spin, args=(node,), daemon=True).start()
client = SwitchBotROSClient(node=node)
print(client.get_devices())
client.control_device('/myroom/light', 'turnOn', wait=True)
```

#### roseus (ROS 1 only)

```lisp
(load "package://switchbot_ros/scripts/switchbot.l")
(setq msg (get-devices)) ;; get device list
(dolist (device (send msg :devices)) (format t "name: ~A, type: ~A~%" (send device :name) (send device :type))) ;; print device list
(control-device "/eng2/7f/73b2/light/upper/switch" "turnOn") ;; control some device
(control-device "/eng2/7f/73b2/light/lower/switch" "turnOn")
```

### An example of controlling the elevator at Eng2 building at The University of Tokyo

Please see [here](https://github.com/sktometometo/jsk_robot/blob/develop/spot/jsk_spot_robot/jsk_spot_behaviors/spot_basic_behaviors/src/spot_basic_behaviors/elevator_behavior.py)
for controlling elevator example code by JSK Spot robot.
