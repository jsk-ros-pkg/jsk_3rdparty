# Google Chat ROS
The ROS wrapper for Google Chat API
1. [Installation Guide](#install)
1. [Sending the message](#send)
1. [Receiving the message](#recieve)
1. [Handling the event](#event)
1. [Optional functions](#optional)
1. [Helper nodes](#helper)

<a id="install"></a>
## 1. Installation Guide
### 1.1 Get the API KEY
At first, you should have the permission to access the Google Chat API.
See [Google Official Document](https://developers.google.com/chat/how-tos/service-accounts#step_1_create_service_account_and_private_key). Please ensure to get JSON credetial file and save it. DO NOT LOST IT!  
For JSK members, all keys are available at [Google Drive](https://drive.google.com/drive/folders/1Enbbta5QuZ-hrUWdTjVDEjDJc3j7Abxo?usp=sharing). If you make new API keys, please upload them here.

### 1.2 Select the way how to recieve Google Chat event
The way you recieve Google Chat event from API server depends on your system. If your system has static IP and is allowed to recieve https request with specific port, please see [HTTPS mode](#https). If not, please see [Pub/Sub mode](#pubsub).

<a id="https"></a>
#### HTTPS mode
When you send the message, the node uses Google REST API. 
When you recieve the message, Google Chat API sends https request to your machine and the node handles it.

![google_chat_https_system](https://user-images.githubusercontent.com/27789460/166410618-6ae286bd-86d8-47e8-87c9-bae0c66493ed.png)

You have to prepare SSL certificate. Self-signed one is not available because of Google security issue. Please use the service like Let's Encrypt. In Google Cloud console, please choose `App URL` as connection settings and fill the URL in the App URL form.

![google_chat_https](https://user-images.githubusercontent.com/27789460/166408349-09520454-4c55-4ca7-bbdc-1d3f27e243b9.png)

<a id="pubsub"></a>
#### Pub/Sub mode
When you send the message, the node uses Google REST API.
When you recieve the message, the node uses Google Pub/Sub API's subscription. The node has already established its connection to Google Pub/Sub API when you launch it.

![google_chat_pubsub_system](https://user-images.githubusercontent.com/27789460/166410714-03f16096-eea4-4eeb-9487-5ab3df309332.png)

The way how to set up in Google Cloud console shows below.
##### 1. Authorize the existing Google Chat API project to access Google Cloud Pub/Sub service  
In IAM settings in the console, please add the role `Pub/Sub Admin` to service account.

![pubsub_admin_mosaic](https://user-images.githubusercontent.com/27789460/166408915-832a279f-da9e-463b-86e4-8a18ad5e4f5a.png)

##### 2. Create Pub/Sub topic and subscriber  
In Pub/Sub settings in the console, please add the topic and subscriptions.
In the figure, we set the topic name `chat`, the subscription name `chat-sub` as an example.

![pubsub_topic_mosaic](https://user-images.githubusercontent.com/27789460/166409434-8f7fa329-1ae1-4cc4-aba2-82f43c2de16f.png)

![pubsub_subscription](https://user-images.githubusercontent.com/27789460/166409454-cc59ec43-f59e-4b63-a1b8-fadefcdd46ce.png)

Note that if you set the topic name `chat`, the full name of it becomes `projects/<project_name>/topics/chat`. Please confirm the subsciptions subscribes the full name not short one.

##### 3. Grant publish rigts on your topic
In order for Google Chat to publish messages to your topic, it must have publishing rights to the topic. To grant Google Chat these permissions, assign the Pub/Sub Publisher role to the following service account
```
chat-api-push@system.gserviceaccount.com
```
![google_chat_pubsub_permission](https://user-images.githubusercontent.com/27789460/173894738-cc169b21-0873-4def-9179-f686a2ae68ec.png)


##### 4. Set Google Chat API Connection settings  
Please choose `Cloud Pub/Sub` as connection settings and fill the full topic name in the Topic Name form.

![google_chat_pubsub](https://user-images.githubusercontent.com/27789460/166408478-b662b73c-35a8-43e8-aaaa-93efdd48e486.png)


### 1.3 Install/Build the ROS node
If you want to build from the source 
```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/jsk-ros-pkg/jsk_3rdparty
rosdep install --ignore-src --from-paths . -y -r
cd ..
catkin build
```
### 1.4 Launch the node
#### HTTPS mode
You have to set rosparams `receiving_mode=https`, `google_cloud_credentials_json`, `host`, `port`, `ssl_certfile`, `ssl_keyfile`.
#### Pub/Sub mode
You have to set rosparams `receiving_mode=pubsub`, `google_cloud_credentials_json`, `project_id`, `subscription_id`. `subscription_id` would be `chat-sub` if you follow [Pub/Sub mode](#pubsub) example. 
##### Example
```bash
roslaunch google_chat_ros google_chat.launch receiving_mode:=pubsub google_cloud_credentials_json:=/path/to/<project_id>-XXXXXXXX.json project_id:=<project_id> subscription_id:=chat-sub
```
<a id="send"></a>
## 2. Sending the message
### 2.1 Understanding Google Chat Room
When you see Google Chat UI with browsers or smartphone's apps, you may see `space`, `thread`. If you send new message, you must specify the space or thread you want to send the message to. You can get the space name from chat room's URL. If it is `https://mail.google.com/chat/u/0/#chat/space/XXXXXXXXXXX`, `XXXXXXXXXXX` becomes the space name.

### 2.2 Message format
There are 2 types of messages, text and card. The card basically follows [the original json structure](https://developers.google.com/chat/api/guides/message-formats/events#event_fields). As the node covers all the units in here with ros action msgs, it may be complicated for you if you want to use all of them. So in Examples sections, we'll show you simple ones.

### 2.3 Sending the message by actionlib
All you have to do is send Actionlib goal to `~send/goal`.

### 2.4 Examples
Showing the message examples with `rostopic pub -1` command on `bash`.
#### Sending a text message
``` bash
rostopic pub -1 /google_chat_ros/send/goal google_chat_ros/SendMessageActionGoal "goal:
  text: 'Hello!'
  space: 'spaces/<space name>'"
```
![google_chat_text](https://user-images.githubusercontent.com/27789460/166410345-4ba29050-a83d-42c7-9dfe-0babc0486001.png)


#### Sending a message with KeyValue card
``` bash
rostopic pub -1 /google_chat_ros/send/goal google_chat_ros/SendMessageActionGoal "goal:
  text: 'Something FATAL errors have happened in my computer, please fix ASAP'
  cards:
    -
      sections:
        -
          widgets:
            -
              key_value:
                top_label: 'Process ID'
                content: '1234'
                bottom_label: 'rospy'
                icon: 'DESCRIPTION'
  space: 'spaces/<space name>'"
```
![google_chat_keyvalue](https://user-images.githubusercontent.com/27789460/166410374-f94a9da7-45fb-4915-929e-3181891d7293.png)


<a id="interactive"></a>
#### Sending an Interactive button
``` bash
rostopic pub -1 /google_chat_ros/send/goal google_chat_ros/SendMessageActionGoal "goal:
  cards:
    -
      header:
        title: 'What do you want to eat?'
        subtitle: 'Please choose the food shop!'
      sections:
        -
          widgets:
            -
              buttons:
                -
                  text_button_name: 'STARBUCKS'
                  text_button_on_click:
                    action:
                      action_method_name: 'vote_starbucks'
                      parameters:
                        -
                          key: 'shop'
                          value: 'starbucks'
                -
                  text_button_name: 'SUBWAY'
                  text_button_on_click:
                    action:
                      action_method_name: 'vote_subway'
                      parameters:
                        -
                          key: 'shop'
                          value: 'subway'

  space: 'spaces/<space name>'"
```
![google_chat_interactive_button](https://user-images.githubusercontent.com/27789460/166410386-395daab4-158c-4f47-b0c3-324b2f258ffd.png)


#### Sending a message with an image
See [Here](#image).

<a id="recieve"></a>
## 3. Receiving the messages
### 3.1 ROS Topic
When the bot was mentioned, the node publishes `~message_activity` topic.

### 3.2 Examples

#### Receiving a text message 
```yaml
event_time: "2022-04-28T06:25:26.884623Z"
space:
  name: "spaces/<space name>"
  display_name: ''
  room: False
  dm: True
message:
  name: "spaces/<space name>/messages/<message id>"
  sender:
    name: "users/<user id>"
    display_name: "Yoshiki Obinata"
    avatar_url: "<avatar url>"
    avatar: []
    email: "<email>"
    bot: False
    human: True
  create_time: "2022-04-28T06:25:26.884623Z"
  text: "Hello!"
  thread_name: "spaces/<space name>/threads/<thread name>"
  annotations: []
  argument_text: "Hello!"
  attachments: []
user:
  name: "users/<user id>"
  display_name: "Yoshiki Obinata"
  avatar_url: "<avatar url>"
  avatar: []
  email: "<email>"
  bot: False
  human: True
```

#### Receiving a message with an image or gdrive file and download it
<!-- TODO add link to example -->

<a id="event"></a>
## 4. Handling the interactive event
If you've already sent the interactive card like [Interactive card example](#interactive), you can receive the activity of buttons. Suppose someone pressed the button `STARBUCKS`, the node publishes a `~card_activity` topic like

``` yaml
event_time: "2022-05-02T00:23:47.855023Z"
space:
  name: "spaces/<space name>"
  display_name: "robotroom_with_thread"
  room: True
  dm: False
message:
  name: "spaces/<space name>/messages/Go__sDfIdec.Go__sDfIdec"
  sender:
    name: "users/100406614699672138585"
    display_name: "Fetch1075"
    avatar_url: "https://lh4.googleusercontent.com/proxy/hWEAWt6fmHsFAzeiEoV5FMOx5-jmU3OnzQxCtrr9unyt73NNwv0lh7InFzOh-0yO3jOPgtColHBywnZnJvl4SVqqqrYkyT1uf18k_hDIVYrAv87AY7lM0hp5KtQ1m9br-aPFE98QwNnSTYc2LQ"
    avatar: []
    email: ''
    bot: True
    human: False
  create_time: "2022-05-02T00:23:47.855023Z"
  text: ''
  thread_name: "spaces/<space name>/threads/Go__sDfIdec"
  annotations: []
  argument_text: ''
  attachments: []
user:
  name: "users/103866924487978823908"
  display_name: "Yoshiki Obinata"
  avatar_url: "https://lh3.googleusercontent.com/a-/AOh14GgexXiq8ImuKMgOq6QG-4geIzz5IC1-xa0Caead=k"
  avatar: []
  email: "<your email>"
  bot: False
  human: True
action:
  action_method_name: "vote_starbucks"
  parameters:
    -
      key: "shop"
      value: "starbucks"
```
After the node which handles the chat event subscribed the topic, it can respond with text message like

``` bash
rostopic pub -1 /google_chat_ros/send/goal google_chat_ros/SendMessageActionGoal "goal:
  cards:
    -
      sections:
        -
          widgets:
            -
              key_value:
                top_label: 'The shop accepted!'
                content: 'You choose STARBUCKS!!'
                icon: 'DESCRIPTION'
  space: 'spaces/<space name>'
  thread_name: 'spaces/<space name>/threads/<thread name>'"
```
![google_chat_interact](https://user-images.githubusercontent.com/27789460/166410418-c2bdc2e5-9916-4b50-a705-f838d86681aa.png)


The important point is that the client node has to remember the `thread_name` which the card event was occured at and send response to it.

<a id="optional"></a>
## 5. Optional functions
<a id="image"></a>
### 5.1 Sending a message with an image
To send an image, you have to use `card` type message. If you want to add the image uploaded to a storage server available for everyone, you just add its URI like
``` yaml
rostopic pub -1 /google_chat_ros/send/goal google_chat_ros/SendMessageActionGoal "goal:
  cards:
    -
      sections:
        -
          widgets:
            -
              image:
                image_url: 'https://media-cdn.tripadvisor.com/media/photo-s/11/fb/90/e4/dsc-7314-largejpg.jpg'
  space: 'spaces/<your space>'"
```
If you want to attach image saved at your host, you have to launch (gdrive_ros)[https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/gdrive_ros] at first and set `~gdrive_upload_service` param with `gdrive_ros/Upload` service name. Then publish topic like 
``` yaml
rostopic pub -1 /google_chat_ros/send/goal google_chat_ros/SendMessageActionGoal "goal:
  cards:
    -
      sections:
        -
          widgets:
            -
              image:
                localpath: '/home/user/Pictures/image.png'
  space: 'spaces/<your space>'
```
### 5.2 Receiving a message with images or gdrive file
You have to set rosparam `~download_data` True, `~download_directory`. If the node recieved the message with image or google drive file, it automatically downloads to `~donwload_directory` path.


### Troubleshoot

#### google.api_core.exceptions.NotFound: 404 Resource not found (resource=chat-sub).

If you encounter `404 Resource not found (resource=chat-sub).` error on your screen as shown in below.

```
[INFO] [1680417167.337634]: [/google_chat_ros] Expected to use Google Cloud Pub Sub service
Traceback (most recent call last):
  File "/home/k-okada/catkin_ws/ws_3rdparty/src/jsk_3rdparty/google_chat_ros/scripts/google_chat_ros_node.py", line 473, in <module>
    node = GoogleChatROS()
  File "/home/k-okada/catkin_ws/ws_3rdparty/src/jsk_3rdparty/google_chat_ros/scripts/google_chat_ros_node.py", line 80,
in __init__
    self._pubsub_client.run()
  File "/home/k-okada/catkin_ws/ws_3rdparty/src/jsk_3rdparty/google_chat_ros/src/google_chat_ros/google_chat.py", line 135, in run
    self._streaming_pull_future.result()
  File "/usr/lib/python3.6/concurrent/futures/_base.py", line 432, in result
    return self.__get_result()
  File "/usr/lib/python3.6/concurrent/futures/_base.py", line 384, in __get_result
    raise self._exception
google.api_core.exceptions.NotFound: 404 Resource not found (resource=chat-sub).
```

Please make sure that you have enabled subscription service, as a default it expires in 7 days. We recommend you to set no expiration date.

![google_chat_ros_404_error](https://user-images.githubusercontent.com/493276/230245558-f85dbde8-c774-4932-ba52-cebf6848bb5a.png)