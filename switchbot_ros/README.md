# switchbot_ros
This is the package for using switchbot by ROS.

## System Component
![Screenshot from 2021-07-17 12-43-06](https://user-images.githubusercontent.com/27789460/126024618-159f57e6-d17e-448f-a606-425b33bc1fff.png)
JSK members can edit the original file [here](https://docs.google.com/presentation/d/18LIq5SMnkFJGJsdDV85V6G5Vyt8o6_0rvvhKwSjwWmk/edit?usp=sharing).

## How to use?
### Setup switchbot
1. Install the switchbot app to your phone(iPhone, Android) and create your switchbot account.  
2. Prepare the switchbot devices and setup by your phone.  

<img src="https://user-images.githubusercontent.com/27789460/121886201-1b0f5700-cd50-11eb-80d2-42cd98554905.PNG" width=150> <img src="https://user-images.githubusercontent.com/27789460/121886458-6164b600-cd50-11eb-81fd-3b9fef35518d.PNG" width=150> <img src="https://user-images.githubusercontent.com/27789460/121886467-6295e300-cd50-11eb-8c33-7fcbcbafb9ac.PNG" width=150> <img src="https://user-images.githubusercontent.com/27789460/121886470-63c71000-cd50-11eb-93a1-274ed98a96e6.PNG" width=150> <img src="https://user-images.githubusercontent.com/27789460/121886477-64f83d00-cd50-11eb-9d51-437b4d18fa8d.PNG" width=150>

Especially, please set your device name and enable cloud service.

3. Get your token  
On switchbot App, profile -> settings, and press `version` for 10 times and you can get token.

For JSK members, please see [this slide](https://docs.google.com/presentation/d/11UkuxVT4u_LcAYJQnPvt4mzxos9Qvflth5hgZmtqmyk/edit?usp=sharing) for account and token details and installed switchbots for our lab.

### How to launch switchbot ros
Execute `roslaunch switchbot_ros switchbot.launch token:=YOUR_TOKEN` and publish ActionGoal.

### ROS API

The `switchbot_ros` node provides APIs below.

#### `~/devices` topic ( Type: `switchbot_ros/DeviceArray` )

This topic provides devices list available with switchbot_ros server.

#### `~/switch` action ( Type: `switchbot_ros/SwitchBotcommandAction` )

This action provide command interface to switchbot devices.

From command line
```bash
rostopic pub /switchbot_ros/switch/goal switchbot_ros/SwitchBotCommandActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  device_name: '/eng2/7f/73b2/test/button'
  command: 'press'
  parameter: ''
  command_type: ''"
```
```bash
rostopic pub /switchbot_ros/switch/goal switchbot_ros/SwitchBotCommandActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  device_name: '/myroom/light'
  command: 'turnOn'
  parameter: ''
  command_type: ''"
```

Please see [here](https://github.com/OpenWonderLabs/SwitchBotAPI#command-set-for-physical-devices) for command details.

### Client libraries

There are rospy/roseus client libraries for `switchbot_ros` server.

#### roseus

```lisp
(setq msg (get-devices)) ;; get device list
(dolist (device (send msg :devices)) (format t "name: ~A, type: ~A~%" (send device :name) (send device :type))) ;; print device list
(control-device "/eng2/7f/73b2/light/upper/switch" "turnOn") ;; control some device
(control-device "/eng2/7f/73b2/light/lower/switch" "turnOn")
```

#### rospy

```python
import rospy
from switchbot_ros.switchbot_ros_client import SwitchBotROSClient

rospy.init_node('hoge')
client = SwitchBotROSClient()

devices = client.get_devices()
print(devices)

client.control_device('/eng2/7f/73b2_bot_kitchen', 'turnOn')
```


### An example of controlling the elevator at Eng2 building at The University of Tokyo
Please see [here](https://github.com/sktometometo/jsk_robot/blob/develop/spot/jsk_spot_robot/jsk_spot_behaviors/spot_basic_behaviors/src/spot_basic_behaviors/elevator_behavior.py) for controlling elevator example code by JSK Spot robot.
