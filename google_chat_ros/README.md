# The package for using Google chat services with ROS

## What is this?
Use Google Chat API with ROS.  
![Screenshot from 2021-11-01 15-53-27](https://user-images.githubusercontent.com/27789460/139635911-66232c88-d3b9-4d7d-940e-966fbac9d800.png)  
System components  
![GoogleChatROS_system](https://user-images.githubusercontent.com/27789460/139635648-4ddbf9da-90e9-4b87-b958-ca996a8ffc4f.png)

## How to use?
### 1. Create a service account and private key
See [Google Official Document](https://developers.google.com/chat/how-tos/service-accounts#step_1_create_service_account_and_private_key). Please ensure to get JSON credetial file and save it. DO NOT LOST IT!  
For JSK members, all keys are available at [Google Drive](https://drive.google.com/drive/folders/1Enbbta5QuZ-hrUWdTjVDEjDJc3j7Abxo?usp=sharing). If you make new  API keys, please upload them here.

### 2. Build ROS workspace
```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/jsk-ros-pkg/jsk_3rdparty
rosdep install --ignore-src --from-paths . -y -r
cd ..
catkin build
```

### 3. Use google chat ros
#### 3.1 Run the server
Execute
```bash
roslaunch google_chat_ros google_chat.launch keyfile:=${PATH_TO_keyfile.json}
```
and run the action server.

#### 2.2 Run the client
First, you have to identify your chat room. You can get it from chat room's URL. If it is `https://mail.google.com/chat/u/0/#chat/space/XXXXXXXXXXX`, `XXXXXXXXXXX` becomes the space name.
##### terminal example
```bash
rostopic pub /google_chat_ros/send/goal google_chat_ros/GoogleChatRESTActionGoal "header:
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
  space: 'YOUR_SPACE'
  message_type: 'text'
  content: 'Hello world from ROS!'"
```
##### roseus example
```lisp
(load "package://google_chat_ros/scripts/google-chat.l")
(send-google-chat-message "YOUR_SPACE" "text" "Hello world from eus!")
```

## Google Chat Message types
You can set Google Chat message type by setting `message_type` in ros message.
### text
Send simple text message.
### card
Send Google Chat Card message.
See [here](https://developers.google.com/chat/api/guides/message-formats/cards) for details.

## Sending Images
You have to get image's permanent link and attach its url to card type message. To get it, please consider using jsk_3rdparty/gdrive_ros.
