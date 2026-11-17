# Google Chat ROS

The ROS wrapper for the Google Chat API: a Google Chat app (bot) which sends text and card messages by an action, and
publishes the messages, space events and card clicks it receives as topics. The same scripts run on ROS 1 and ROS 2.

1. [Requirements](#requirements)
1. [Usage](#usage)
1. [Sending the message](#send)
1. [Receiving the message](#recieve)
1. [Handling the interactive event](#event)
1. [Optional functions](#optional)
1. [Interface](#interface)
1. [Troubleshooting](#troubleshoot)

<a id="requirements"></a>
## 1. Requirements

### 1.1 Get the API key

The node needs the JSON key file of a Google Cloud service account which can use the Google Chat API, given by
`google_cloud_credentials_json`. See the
[Google document](https://developers.google.com/chat/how-tos/service-accounts#step_1_create_service_account_and_private_key):
create a Google Cloud project, enable the Google Chat API, configure the Chat app, create a service account and download
its JSON key. Keep the key private.
For JSK members, all keys are available at [Google Drive](https://drive.google.com/drive/folders/1Enbbta5QuZ-hrUWdTjVDEjDJc3j7Abxo?usp=sharing). If you make new API keys, please upload them there.

Without a valid key the node still starts, but it does not provide the `~send` action (it logs
`Failed to start Google Chat REST service`); receiving in `url` mode works without a key.

### 1.2 Select the way how to receive Google Chat events

The way you receive Google Chat events from the API server depends on your system. If your system has a static IP and
is allowed to receive HTTPS requests on a port, use [HTTPS mode](#https) (`receiving_mode:=url`). If not, use
[Pub/Sub mode](#pubsub) (`receiving_mode:=pubsub`). `receiving_mode:=none` only sends messages.

<a id="https"></a>
#### HTTPS mode
When you send the message, the node uses the Google REST API.
When you receive the message, the Google Chat API sends an HTTPS request to your machine and the node handles it.

![google_chat_https_system](https://user-images.githubusercontent.com/27789460/166410618-6ae286bd-86d8-47e8-87c9-bae0c66493ed.png)

You have to prepare an SSL certificate. A self-signed one is not accepted by Google. Please use a service like Let's
Encrypt. In the Google Cloud console, choose `App URL` as the connection setting and fill the URL in the App URL form.

![google_chat_https](https://user-images.githubusercontent.com/27789460/166408349-09520454-4c55-4ca7-bbdc-1d3f27e243b9.png)

<a id="pubsub"></a>
#### Pub/Sub mode
When you send the message, the node uses the Google REST API.
When you receive the message, the node uses a Google Pub/Sub subscription. The node establishes its connection to
the Google Pub/Sub API when it is launched.

![google_chat_pubsub_system](https://user-images.githubusercontent.com/27789460/166410714-03f16096-eea4-4eeb-9487-5ab3df309332.png)

The setup in the Google Cloud console is as follows.

##### 1. Authorize the Google Chat API project to access the Google Cloud Pub/Sub service
In the IAM settings in the console, add the role `Pub/Sub Admin` to the service account.

![pubsub_admin_mosaic](https://user-images.githubusercontent.com/27789460/166408915-832a279f-da9e-463b-86e4-8a18ad5e4f5a.png)

##### 2. Create a Pub/Sub topic and subscription
In the Pub/Sub settings in the console, add the topic and the subscription.
In the figures, the topic name is `chat` and the subscription name is `chat-sub`.

![pubsub_topic_mosaic](https://user-images.githubusercontent.com/27789460/166409434-8f7fa329-1ae1-4cc4-aba2-82f43c2de16f.png)

![pubsub_subscription](https://user-images.githubusercontent.com/27789460/166409454-cc59ec43-f59e-4b63-a1b8-fadefcdd46ce.png)

If you set the topic name `chat`, its full name becomes `projects/<project_name>/topics/chat`. Make sure the
subscription subscribes to the full name, not the short one. Set the subscription to never expire (see
[Troubleshooting](#troubleshoot)).

##### 3. Grant publish rights on your topic
For Google Chat to publish messages to your topic, assign the Pub/Sub Publisher role to the following service account:
```
chat-api-push@system.gserviceaccount.com
```
![google_chat_pubsub_permission](https://user-images.githubusercontent.com/27789460/173894738-cc169b21-0873-4def-9179-f686a2ae68ec.png)

##### 4. Set the Google Chat API connection settings
Choose `Cloud Pub/Sub` as the connection setting and fill the full topic name in the Topic Name form.

![google_chat_pubsub](https://user-images.githubusercontent.com/27789460/166408478-b662b73c-35a8-43e8-aaaa-93efdd48e486.png)

### 1.3 Python dependencies

The Google API, Pub/Sub and gdown python modules are installed at build time: on ROS 1 into a virtualenv by
`catkin_virtualenv` (`requirements.in`, or `requirements.in.python3.12` on Python >= 3.12), on ROS 2 into a uv venv
(`share/google_chat_ros/venv`) from `pyproject.toml`/`uv.lock` (`jsk_ros1_ros2_compat_generate_uv_venv()`). Run
`uv lock` after changing `pyproject.toml`.

<a id="usage"></a>
## 2. Usage

### ROS 2

```bash
# Pub/Sub mode
ros2 launch google_chat_ros google_chat.launch.xml receiving_mode:=pubsub \
  google_cloud_credentials_json:=/path/to/<project_id>-XXXXXXXX.json \
  project_id:=<project_id> subscription_id:=chat-sub
# HTTPS mode
ros2 launch google_chat_ros google_chat.launch.xml receiving_mode:=url \
  google_cloud_credentials_json:=/path/to/key.json \
  host:=0.0.0.0 port:=443 ssl_certfile:=/path/to/fullchain.pem ssl_keyfile:=/path/to/privkey.pem
```

Send a message and watch the received ones:

```bash
ros2 action send_goal /google_chat_ros/send google_chat_ros/action/SendMessage \
  "{text: 'Hello!', space: 'spaces/<space name>'}"
ros2 topic echo /google_chat_ros/message_activity
```

### ROS 1

```bash
roslaunch google_chat_ros google_chat.launch receiving_mode:=pubsub \
  google_cloud_credentials_json:=/path/to/<project_id>-XXXXXXXX.json \
  project_id:=<project_id> subscription_id:=chat-sub
```

```bash
rostopic pub -1 /google_chat_ros/send/goal google_chat_ros/SendMessageActionGoal "goal:
  text: 'Hello!'
  space: 'spaces/<space name>'"
rostopic echo /google_chat_ros/message_activity
```

EusLisp functions to send messages (`send-google-chat-text`, `send-google-chat-image`, `send-google-chat-buttons`)
are in `scripts/google-chat.l` (ROS 1 only).

### Launch arguments

`launch/google_chat.launch` (ROS 1) and `launch/google_chat.launch.xml` (ROS 2):

| Argument | Default | Description |
|---|---|---|
| `receiving_mode` | `url` | `url` (HTTPS mode), `pubsub`, `dialogflow` or `none` |
| `google_cloud_credentials_json` | `''` | The JSON key file of the service account |
| `gdrive_upload_service` | ROS 1: `/gdrive_ros/upload`, ROS 2: `/gdrive_server/upload` | The `gdrive_ros/Upload` service, to send local images. The server of [gdrive_ros](../gdrive_ros) is `/gdrive_server/upload` by default, so set this on ROS 1 |
| `upload_data_timeout` | `20` | The timeout [s] of the upload |
| `download_data` | `true` | Download the attachments of the received messages |
| `download_data_timeout` | `10` | Set as a parameter, but not used by the node |
| `download_directory` | `/tmp` | The directory of the downloaded attachments |
| `download_avatar` | `false` | Download the avatar images of the users into `User/avatar` |
| `host` | `localhost` | HTTPS mode: the address of the HTTPS server |
| `port` | `443` | HTTPS mode: the port of the HTTPS server |
| `ssl_certfile` | `default` | HTTPS mode: the certificate file |
| `ssl_keyfile` | `default` | HTTPS mode: the private key file |
| `project_id` | `default` | Pub/Sub mode: the Google Cloud project ID |
| `subscription_id` | `default` | Pub/Sub mode: the subscription ID (`chat-sub` in the example above) |
| `use_yaml` | `false` | Load the parameters of the node from `yaml_file` too |
| `yaml_file` | `''` | ROS 1: a rosparam yaml; ROS 2: a ROS 2 parameter file (`/**: ros__parameters: ...`) |
| `respawn` | `true` | Respawn the nodes |
| `use_helper` | `true` | Start `helper.py` |
| `to_dialogflow_client` | `false` | helper: answer the messages with dialogflow_task_executive (ROS 1 only) |
| `debug_sound` | `false` | helper: say the received messages with sound_play (`robotsound_jp`) |

### Trying the receiver locally

The HTTPS server can be tried without Google: with a self-signed certificate and no key, post an event as Google
Chat does (the server handles only requests with the user agent `Google-Dynamite`).

```bash
openssl req -x509 -newkey rsa:2048 -nodes -days 1 -subj /CN=localhost \
  -keyout /tmp/chat.key -out /tmp/chat.crt
ros2 launch google_chat_ros google_chat.launch.xml receiving_mode:=url port:=8443 \
  ssl_certfile:=/tmp/chat.crt ssl_keyfile:=/tmp/chat.key
# another terminal
ros2 topic echo /google_chat_ros/message_activity
# another terminal
curl -k -A Google-Dynamite https://localhost:8443/ -H 'Content-Type: application/json' -d '{
  "type": "MESSAGE", "eventTime": "2022-04-28T06:25:26.884623Z",
  "space": {"name": "spaces/AAAA", "type": "DM"},
  "user": {"name": "users/1", "displayName": "Alice", "type": "HUMAN"},
  "message": {"name": "spaces/AAAA/messages/1", "text": "Hello!", "argumentText": "Hello!",
              "sender": {"name": "users/1", "displayName": "Alice", "type": "HUMAN"},
              "thread": {"name": "spaces/AAAA/threads/1"}}}'
```

<a id="send"></a>
## 3. Sending the message
### 3.1 Understanding Google Chat rooms
In the Google Chat UI of the browser or the smartphone app, you see `space` and `thread`. To send a new message, you
specify the space or thread to send the message to. You can get the space name from the URL of the chat room: if it
is `https://mail.google.com/chat/u/0/#chat/space/XXXXXXXXXXX`, `XXXXXXXXXXX` is the space name, and the `space` of the
goal is `spaces/XXXXXXXXXXX`.

### 3.2 Message format
There are 2 types of messages, text and card. The card follows
[the original json structure](https://developers.google.com/chat/api/reference/rest/v1/cards). The goal of the action
covers all the units of it with ROS messages, so the examples below show simple ones.

### 3.3 Sending the message by the action
Send a goal of `google_chat_ros/SendMessage` to the action `~send` (`/google_chat_ros/send`). The examples below are
of ROS 1 (`rostopic pub` to `~send/goal`); on ROS 2 pass the same fields, without the `goal:` level, to
`ros2 action send_goal /google_chat_ros/send google_chat_ros/action/SendMessage "<fields>"`.

### 3.4 Examples
#### Sending a text message
``` bash
rostopic pub -1 /google_chat_ros/send/goal google_chat_ros/SendMessageActionGoal "goal:
  text: 'Hello!'
  space: 'spaces/<space name>'"
```
![google_chat_text](https://user-images.githubusercontent.com/27789460/166410345-4ba29050-a83d-42c7-9dfe-0babc0486001.png)

#### Sending a message with a KeyValue card
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
#### Sending an interactive button
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

Only text buttons (`text_button_name`) are supported; image buttons (`image_button_name`) are not sent.

#### Sending a message with an image
See [here](#image).

<a id="recieve"></a>
## 4. Receiving the messages
### 4.1 ROS topics
When the bot is mentioned (or messaged in a DM), the node publishes `~message_activity`
(`google_chat_ros/MessageEvent`). Adding or removing the bot to a space publishes `~space_activity`
(`google_chat_ros/SpaceEvent`), and a click on a card button publishes `~card_activity` (`google_chat_ros/CardEvent`).

### 4.2 Example
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

<a id="event"></a>
## 5. Handling the interactive event
After sending an interactive card like the [interactive card example](#interactive), you receive the activity of its
buttons. When someone presses the button `STARBUCKS`, the node publishes `~card_activity` like

``` yaml
event_time: "2022-05-02T00:23:47.855023Z"
space:
  name: "spaces/<space name>"
  display_name: "robotroom_with_thread"
  room: True
  dm: False
message:
  name: "spaces/<space name>/messages/<message id>"
  sender:
    name: "users/<bot user id>"
    display_name: "Fetch1075"
    avatar_url: "<avatar url>"
    avatar: []
    email: ''
    bot: True
    human: False
  create_time: "2022-05-02T00:23:47.855023Z"
  text: ''
  thread_name: "spaces/<space name>/threads/<thread name>"
  annotations: []
  argument_text: ''
  attachments: []
user:
  name: "users/<user id>"
  display_name: "Yoshiki Obinata"
  avatar_url: "<avatar url>"
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
The node which handles the event can respond with a message in the same thread:

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

The client node has to remember the `thread_name` of the card event and send the response to it.

<a id="optional"></a>
## 6. Optional functions
<a id="image"></a>
### 6.1 Sending a message with an image
To send an image, use a `card` message. For an image on a server open to everyone, add its URL:
``` bash
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
For an image on your host, launch [gdrive_ros](../gdrive_ros) first and set `gdrive_upload_service` to its
`gdrive_ros/Upload` service (`/gdrive_server/upload` by default). The node uploads the file into the Google Drive
folder `chat_notification/<timestamp>` and sends its URL:
``` bash
rostopic pub -1 /google_chat_ros/send/goal google_chat_ros/SendMessageActionGoal "goal:
  cards:
    -
      sections:
        -
          widgets:
            -
              image:
                localpath: '/home/user/Pictures/image.png'
  space: 'spaces/<your space>'"
```
`header.image_filepath` of a card and `key_value.original_icon_localpath` are uploaded the same way.

### 6.2 Receiving a message with images or Google Drive files
With `download_data:=true`, the attachments of the received messages are downloaded (with gdown) into
`download_directory`.

<a id="interface"></a>
## 7. Interface

### google_chat_ros_node.py

#### Actions

- `~send` (`google_chat_ros/SendMessage`): send a message. Goal: `text`, `cards` (`google_chat_ros/Card[]`),
  `update_message` (update the message of the card instead of a new one), `thread_name`, `space` (must begin with
  `spaces/`). Result: `message_result` (`google_chat_ros/Message`), `done` (bool). Feedback: `status` (the error).
  Provided only when the service account key is valid.

#### Published topics

Only in the `url`, `pubsub` and `dialogflow` modes:

- `~message_activity` (`google_chat_ros/MessageEvent`)
- `~space_activity` (`google_chat_ros/SpaceEvent`)
- `~card_activity` (`google_chat_ros/CardEvent`)

#### Parameters

- `~receiving_mode` (string, required): `url`, `pubsub`, `dialogflow` or `none`.
- `~google_cloud_credentials_json` (string, required): the JSON key file of the service account.
- `~gdrive_upload_service` (string, default: `/gdrive_ros/upload`): the `gdrive_ros/Upload` service.
- `~upload_data_timeout` (int, default: `30`): the timeout [s] of the upload.
- `~upload_parents_path` (string, default: `chat_notification`): the Google Drive folder of the uploaded files.
- `~download_data` (bool, default: `true`): download the attachments of the received messages.
- `~download_directory` (string, default: `/tmp`): the directory of the downloaded attachments.
- `~download_avatar` (bool, default: `false`): download the avatar images of the users.
- `~host`, `~port`, `~ssl_certfile`, `~ssl_keyfile` (required in `url` mode): the HTTPS server.
- `~project_id`, `~subscription_id` (string, required in `pubsub` mode): the Pub/Sub subscription.

### helper.py

Optional reactions to the received messages.

#### Subscribed topics

- `google_chat_ros/message_activity` (`google_chat_ros/MessageEvent`)

#### Action clients

- `google_chat_ros/send` (`google_chat_ros/SendMessage`): the answer of dialogflow (`to_dialogflow_client`).
- `dialogflow_client/text_action` (`dialogflow_task_executive/DialogText`): ROS 1 only; dialogflow_task_executive is
  not available on ROS 2 yet, and `to_dialogflow_client` is ignored there.
- `robotsound_jp` (`sound_play/SoundRequest`, `sound_play_msgs/action/SoundRequest` on ROS 2): `debug_sound`.

#### Parameters

- `~to_dialogflow_client` (bool, required): send the text of the received messages to dialogflow and answer in
  the thread.
- `~debug_sound` (bool, required): say "<sender>さんから，<text>というメッセージを受信しました".

<a id="troubleshoot"></a>
## 8. Troubleshooting

### google.api_core.exceptions.NotFound: 404 Resource not found (resource=chat-sub).

If you encounter the `404 Resource not found (resource=chat-sub).` error as shown below,

```
[INFO] [1680417167.337634]: [/google_chat_ros] Expected to use Google Cloud Pub Sub service
Traceback (most recent call last):
  ...
  File ".../google_chat_ros/src/google_chat_ros/google_chat.py", line 135, in run
    self._streaming_pull_future.result()
  ...
google.api_core.exceptions.NotFound: 404 Resource not found (resource=chat-sub).
```

make sure that the subscription is enabled: by default it expires in 7 days. We recommend setting no expiration date.

![google_chat_ros_404_error](https://user-images.githubusercontent.com/493276/230245558-f85dbde8-c774-4932-ba52-cebf6848bb5a.png)
