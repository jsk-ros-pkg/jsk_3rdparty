# The package for using Google chat services with ROS

## What is this?
Use Google Chat API with ROS.  
![Screenshot from 2021-11-01 15-53-27](https://user-images.githubusercontent.com/27789460/139635911-66232c88-d3b9-4d7d-940e-966fbac9d800.png)  
System components  
![GoogleChatROS_system](https://user-images.githubusercontent.com/27789460/139635648-4ddbf9da-90e9-4b87-b958-ca996a8ffc4f.png)


## Create a service account and private key
See [Google Official Document](https://developers.google.com/chat/how-tos/service-accounts#step_1_create_service_account_and_private_key). Please ensure to get JSON credetial file and save it. DO NOT LOST IT!

## Use google chat ros
### Run the server
Execute
```bash
roslaunch roslaunch google_chat_ros google_chat.launch keyfile:=${PATH_TO_keyfile.json}
```
and run the action server.

### Clients
First, you have to identify your chat room ID. You can get it from chat room's URL. If it is `https://mail.google.com/chat/u/0/#chat/space/XXXXXXXXXXX`, `XXXXXXXXXXX` becomes the ID.
#### From terminal
```bash
rostopic pub /google_chat_ros/rest/goal google_chat_ros/GoogleChatRESTActionGoal "header:
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
  space: 'AAAAaXKY29M'
  message_type: 'text'
  content: 'Hello world from ROS!'"
```
#### From roseus
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


