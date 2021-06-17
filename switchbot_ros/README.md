# switchbot_ros
This is the package for using switchbot by ROS.

## How to use?
### Setup switchbot
1. Install the switchbot app to your phone(iPhone, Android) and create your switchbot account.  
2. Prepare the switchbot devices and setup by your phone.  

<img src="https://user-images.githubusercontent.com/27789460/121886201-1b0f5700-cd50-11eb-80d2-42cd98554905.PNG" width=150> <img src="https://user-images.githubusercontent.com/27789460/121886458-6164b600-cd50-11eb-81fd-3b9fef35518d.PNG" width=150> <img src="https://user-images.githubusercontent.com/27789460/121886467-6295e300-cd50-11eb-8c33-7fcbcbafb9ac.PNG" width=150> <img src="https://user-images.githubusercontent.com/27789460/121886470-63c71000-cd50-11eb-93a1-274ed98a96e6.PNG" width=150> <img src="https://user-images.githubusercontent.com/27789460/121886477-64f83d00-cd50-11eb-9d51-437b4d18fa8d.PNG" width=150>

Especially, please set your device name and enable cloud service.

3. Get your token  
On switchbot App, profile -> settings, and press `version` for 10 times and you can get token. For JSK members, we already have shared one [here](https://drive.google.com/file/d/1YZ4P4aaPemB_umB9S0xDG66BJt59-2lz/view?usp=sharing).

### Using switchbot ros
Execute `roslaunch switchbot_ros switchbot.launch token:=YOUR_TOKEN` and publish ActionGoal.

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

From roseus
```
(control-device "/eng2/7f/73b2/light/upper/switch" "turnOn")
(control-device "/eng2/7f/73b2/light/lower/switch" "turnOn")
```

Please see [here](https://github.com/OpenWonderLabs/SwitchBotAPI#command-set-for-physical-devices) for command details.
