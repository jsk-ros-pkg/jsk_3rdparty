# Google Chat ROS

A Google Chat app (bot) for ROS. `google_chat_ros_node.py`

- **sends** text and card messages to a Google Chat space: goals of the action `/google_chat_ros/send`;
- **receives** the messages to the bot, the bot being added to / removed from a space, and the clicks on card buttons,
  and publishes them on `/google_chat_ros/message_activity`, `/google_chat_ros/space_activity` and
  `/google_chat_ros/card_activity`.

The same launch files and scripts work on ROS 1 and ROS 2.

1. [Choose a mode](#modes)
1. [Quick start (Pub/Sub mode, ROS 2)](#quickstart)
1. [Sending messages](#send)
1. [Receiving messages](#receive)
1. [Mode details](#mode-details): [Pub/Sub](#pubsub), [HTTPS](#https), [sending only](#none), [Dialogflow](#dialogflow)
1. [ROS 1](#ros1)
1. [Interface](#interface)
1. [Troubleshooting](#troubleshoot)
1. [Notes](#notes)

<a id="modes"></a>
## 1. Choose a mode

Sending always uses the Google Chat REST API. How the node receives events is chosen by `receiving_mode`:

| Mode | `receiving_mode` | Sends | Receives | What you need |
|---|---|:---:|:---:|---|
| [Pub/Sub](#pubsub) (recommended) | `pubsub` | ✓ | ✓ | Service account key; Chat app with the connection "Cloud Pub/Sub"; a Pub/Sub topic and subscription. No public address needed |
| [HTTPS](#https) | `url` (default) | ✓ (with a key) | ✓ | Chat app with the connection "App URL"; a public HTTPS endpoint (reachable host and port, certificate signed by a CA, e.g. Let's Encrypt; not self-signed); the key only to send |
| [Sending only](#none) | `none` | ✓ | | Service account key; Chat app configured |
| [Dialogflow](#dialogflow) | `dialogflow` | ✓ | | Not a receiving mode any more: it creates the topics but receives nothing. Use `pubsub` or `url` and the helper (`to_dialogflow_client`, ROS 1 only) |

The **service account key** is the JSON key file of a Google Cloud service account which can use the Google Chat API
(launch argument `google_cloud_credentials_json`). Without a valid key the node starts, but it does not provide the
`send` action (it logs `Failed to start Google Chat REST service`). For JSK members, the keys are on
[Google Drive](https://drive.google.com/drive/folders/1Enbbta5QuZ-hrUWdTjVDEjDJc3j7Abxo?usp=sharing); if you make a new
key, upload it there. Keep the keys private.

<a id="quickstart"></a>
## 2. Quick start (Pub/Sub mode, ROS 2)

1. **Google Cloud project and key.** Create a project, enable the Google Chat API, create a service account and
   download its JSON key (see the
   [Google document](https://developers.google.com/chat/how-tos/service-accounts#step_1_create_service_account_and_private_key)).
1. **Pub/Sub.** In the console of the project:
   1. IAM: give the service account the role `Pub/Sub Admin`.
   1. Pub/Sub: create a topic (e.g. `chat`, full name `projects/<project_id>/topics/chat`) and a subscription of it
      (e.g. `chat-sub`) which **never expires** (by default it expires after a period of inactivity, see
      [Troubleshooting](#troubleshoot)).
   1. On the topic, give the role `Pub/Sub Publisher` to `chat-api-push@system.gserviceaccount.com`, so that Google
      Chat can publish the events.

   Screenshots: [Pub/Sub mode](#pubsub).
1. **Chat app.** In the configuration of the Google Chat API, set the app name, avatar and description, and choose
   the connection setting `Cloud Pub/Sub` with the full topic name `projects/<project_id>/topics/chat`.
1. **Build** (the Python modules are installed into a venv of the package at build time):
   ```bash
   colcon build --packages-up-to google_chat_ros
   ```
1. **Launch:**
   ```bash
   ros2 launch google_chat_ros google_chat.launch.xml receiving_mode:=pubsub \
     google_cloud_credentials_json:=/path/to/<project_id>-XXXXXXXX.json \
     project_id:=<project_id> subscription_id:=chat-sub
   ```
1. **Add the app to a space** in Google Chat (or open a DM with it), and get the space name from the URL of the space:
   `https://mail.google.com/chat/u/0/#chat/space/XXXXXXXXXXX` is the space `spaces/XXXXXXXXXXX`.
1. **Receive:** mention the app in the space (or write in the DM), and see the message:
   ```bash
   ros2 topic echo /google_chat_ros/message_activity
   ```
1. **Send:**
   ```bash
   ros2 action send_goal /google_chat_ros/send google_chat_ros/action/SendMessage \
     "{text: 'Hello!', space: 'spaces/XXXXXXXXXXX'}"
   ```

<a id="send"></a>
## 3. Sending messages

Send a goal of `google_chat_ros/action/SendMessage` to `/google_chat_ros/send`. `space` (beginning with `spaces/`) is
required; give `thread_name` (`spaces/<space>/threads/<thread>`) to answer in a thread. A message has a `text`, `cards`
or both; the cards follow [the cards v1 json](https://developers.google.com/chat/api/reference/rest/v1/cards), each
unit of which is a ROS message of this package. On ROS 1, publish the same fields under `goal:` to
`/google_chat_ros/send/goal` (see [ROS 1](#ros1)).

#### Text

```bash
ros2 action send_goal /google_chat_ros/send google_chat_ros/action/SendMessage \
  "{text: 'Hello!', space: 'spaces/<space>'}"
```
![google_chat_text](https://user-images.githubusercontent.com/27789460/166410345-4ba29050-a83d-42c7-9dfe-0babc0486001.png)

#### A KeyValue card

```bash
ros2 action send_goal /google_chat_ros/send google_chat_ros/action/SendMessage "
text: 'Something FATAL errors have happened in my computer, please fix ASAP'
cards:
  - sections:
      - widgets:
          - key_value:
              top_label: 'Process ID'
              content: '1234'
              bottom_label: 'rospy'
              icon: 'DESCRIPTION'
space: 'spaces/<space>'"
```
![google_chat_keyvalue](https://user-images.githubusercontent.com/27789460/166410374-f94a9da7-45fb-4915-929e-3181891d7293.png)

<a id="interactive"></a>
#### Buttons

```bash
ros2 action send_goal /google_chat_ros/send google_chat_ros/action/SendMessage "
cards:
  - header:
      title: 'What do you want to eat?'
      subtitle: 'Please choose the food shop!'
    sections:
      - widgets:
          - buttons:
              - text_button_name: 'STARBUCKS'
                text_button_on_click:
                  action:
                    action_method_name: 'vote_starbucks'
                    parameters: [{key: 'shop', value: 'starbucks'}]
              - text_button_name: 'SUBWAY'
                text_button_on_click:
                  action:
                    action_method_name: 'vote_subway'
                    parameters: [{key: 'shop', value: 'subway'}]
space: 'spaces/<space>'"
```
![google_chat_interactive_button](https://user-images.githubusercontent.com/27789460/166410386-395daab4-158c-4f47-b0c3-324b2f258ffd.png)

Only text buttons (`text_button_name`) are sent; image buttons (`image_button_name`) are not. A click is published on
`card_activity` ([Card clicks](#card-clicks)).

<a id="image"></a>
#### Images

An image on a server open to everyone, by its URL:

```bash
ros2 action send_goal /google_chat_ros/send google_chat_ros/action/SendMessage "
cards:
  - sections:
      - widgets:
          - image:
              image_url: 'https://media-cdn.tripadvisor.com/media/photo-s/11/fb/90/e4/dsc-7314-largejpg.jpg'
space: 'spaces/<space>'"
```

A file on your host (`localpath`): launch [gdrive_ros](../gdrive_ros) and give its `gdrive_ros/Upload` service by
`gdrive_upload_service` (default `/gdrive_server/upload` on ROS 2, which is the default of gdrive_ros;
`/gdrive_ros/upload` on ROS 1). The node uploads the file into the Google Drive folder
`chat_notification/<timestamp>` and sends its URL. `header.image_filepath` of a card and
`key_value.original_icon_localpath` are uploaded the same way.

```bash
ros2 action send_goal /google_chat_ros/send google_chat_ros/action/SendMessage "
cards:
  - sections:
      - widgets:
          - image:
              localpath: '/home/user/Pictures/image.png'
space: 'spaces/<space>'"
```

EusLisp functions (`send-google-chat-text`, `send-google-chat-image`, `send-google-chat-buttons`) are in
`scripts/google-chat.l` (ROS 1 only).

<a id="receive"></a>
## 4. Receiving messages

In the `pubsub` and `url` modes, the node publishes

- `~message_activity` (`google_chat_ros/MessageEvent`) when the app is mentioned in a space or messaged in a DM;
- `~space_activity` (`google_chat_ros/SpaceEvent`) when the app is added to or removed from a space;
- `~card_activity` (`google_chat_ros/CardEvent`) when a button of a card of the app is clicked.

With `download_data:=true` (default), the attachments of the received messages are downloaded (with gdown) into
`download_directory` (`/tmp`).

A received text message:

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
  ...
```

<a id="card-clicks"></a>
### Card clicks

When someone presses `STARBUCKS` of the [buttons example](#interactive), `~card_activity` has the `space`, the
`message` of the card (its `thread_name`), the `user` who clicked, and

```yaml
action:
  action_method_name: "vote_starbucks"
  parameters:
    -
      key: "shop"
      value: "starbucks"
```

To answer in the same thread, remember the `thread_name` of the event and send to it:

```bash
ros2 action send_goal /google_chat_ros/send google_chat_ros/action/SendMessage "
cards:
  - sections:
      - widgets:
          - key_value:
              top_label: 'The shop accepted!'
              content: 'You choose STARBUCKS!!'
              icon: 'DESCRIPTION'
space: 'spaces/<space name>'
thread_name: 'spaces/<space name>/threads/<thread name>'"
```
![google_chat_interact](https://user-images.githubusercontent.com/27789460/166410418-c2bdc2e5-9916-4b50-a705-f838d86681aa.png)

<a id="mode-details"></a>
## 5. Mode details

<a id="pubsub"></a>
### Pub/Sub mode (`receiving_mode:=pubsub`)

The node pulls the events from a Pub/Sub subscription, with the service account key; it connects to Pub/Sub when it
starts. Nothing has to be reachable from the internet.

![google_chat_pubsub_system](https://user-images.githubusercontent.com/27789460/166410714-03f16096-eea4-4eeb-9487-5ab3df309332.png)

The setup of [the quick start](#quickstart) in the console:

1. The role `Pub/Sub Admin` of the service account (IAM):

   ![pubsub_admin_mosaic](https://user-images.githubusercontent.com/27789460/166408915-832a279f-da9e-463b-86e4-8a18ad5e4f5a.png)
1. The topic (`chat`) and the subscription (`chat-sub`). The subscription subscribes to the full name of the topic,
   `projects/<project_id>/topics/chat`, not the short one:

   ![pubsub_topic_mosaic](https://user-images.githubusercontent.com/27789460/166409434-8f7fa329-1ae1-4cc4-aba2-82f43c2de16f.png)
   ![pubsub_subscription](https://user-images.githubusercontent.com/27789460/166409454-cc59ec43-f59e-4b63-a1b8-fadefcdd46ce.png)
1. The role `Pub/Sub Publisher` of `chat-api-push@system.gserviceaccount.com` on the topic:

   ![google_chat_pubsub_permission](https://user-images.githubusercontent.com/27789460/173894738-cc169b21-0873-4def-9179-f686a2ae68ec.png)
1. The connection setting `Cloud Pub/Sub` of the Chat app, with the full topic name:

   ![google_chat_pubsub](https://user-images.githubusercontent.com/27789460/166408478-b662b73c-35a8-43e8-aaaa-93efdd48e486.png)

Launch arguments: `project_id`, `subscription_id` (the short name, e.g. `chat-sub`), `google_cloud_credentials_json`.

<a id="https"></a>
### HTTPS mode (`receiving_mode:=url`)

Google Chat sends each event as an HTTPS POST to the node, which runs an HTTPS server on `host`:`port`. The machine
must be reachable from the internet (e.g. a static IP and an open port), with a certificate signed by a CA (Google
does not accept a self-signed one; use e.g. Let's Encrypt). The server handles only the requests with the user agent
`Google-Dynamite` (that of Google Chat). The key is needed only to send.

![google_chat_https_system](https://user-images.githubusercontent.com/27789460/166410618-6ae286bd-86d8-47e8-87c9-bae0c66493ed.png)

In the configuration of the Chat app, choose the connection setting `App URL` and enter the URL of the server:

![google_chat_https](https://user-images.githubusercontent.com/27789460/166408349-09520454-4c55-4ca7-bbdc-1d3f27e243b9.png)

```bash
ros2 launch google_chat_ros google_chat.launch.xml receiving_mode:=url \
  google_cloud_credentials_json:=/path/to/key.json \
  host:=0.0.0.0 port:=443 ssl_certfile:=/path/to/fullchain.pem ssl_keyfile:=/path/to/privkey.pem
```

`host` defaults to `localhost`, which is not reachable from Google: give `0.0.0.0` (or the address of the interface).

#### Trying the receiver locally

The HTTPS server can be tried without Google: with a self-signed certificate and no key, post an event as Google Chat
does.

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

<a id="none"></a>
### Sending only (`receiving_mode:=none`)

Only the `send` action; no topics are published.

```bash
ros2 launch google_chat_ros google_chat.launch.xml receiving_mode:=none \
  google_cloud_credentials_json:=/path/to/key.json
```

<a id="dialogflow"></a>
### Dialogflow

`receiving_mode:=dialogflow` is still accepted, but its receiver (a subscriber of the Dialogflow webhook) was removed:
the node creates the three topics and publishes nothing on them. To answer the messages with Dialogflow, receive them
in the `pubsub` or `url` mode and launch the helper with `to_dialogflow_client:=true` (ROS 1 only, as
[dialogflow_task_executive](../dialogflow_task_executive) is not on ROS 2): `helper.py` sends the text of each
received message to the action `dialogflow_client/text_action` and answers its response in the thread.

<a id="ros1"></a>
## 6. ROS 1

```bash
roslaunch google_chat_ros google_chat.launch receiving_mode:=pubsub \
  google_cloud_credentials_json:=/path/to/<project_id>-XXXXXXXX.json \
  project_id:=<project_id> subscription_id:=chat-sub
rostopic echo /google_chat_ros/message_activity
rostopic pub -1 /google_chat_ros/send/goal google_chat_ros/SendMessageActionGoal "goal:
  text: 'Hello!'
  space: 'spaces/<space name>'"
```

The goals of [Sending messages](#send) are the same, indented under `goal:`. Set
`gdrive_upload_service:=/gdrive_server/upload` to send local images with the default server of gdrive_ros.

<a id="interface"></a>
## 7. Interface

### Launch arguments

`launch/google_chat.launch` (ROS 1) and `launch/google_chat.launch.xml` (ROS 2):

| Argument | Default | Description |
|---|---|---|
| `receiving_mode` | `url` | `pubsub`, `url` (HTTPS), `none` or `dialogflow` ([modes](#modes)) |
| `google_cloud_credentials_json` | `''` | The JSON key file of the service account |
| `project_id` | `default` | Pub/Sub mode: the Google Cloud project ID |
| `subscription_id` | `default` | Pub/Sub mode: the subscription ID (e.g. `chat-sub`) |
| `host` | `localhost` | HTTPS mode: the address of the HTTPS server |
| `port` | `443` | HTTPS mode: the port of the HTTPS server |
| `ssl_certfile` | `default` | HTTPS mode: the certificate file |
| `ssl_keyfile` | `default` | HTTPS mode: the private key file |
| `gdrive_upload_service` | ROS 1: `/gdrive_ros/upload`, ROS 2: `/gdrive_server/upload` | The `gdrive_ros/Upload` service, to send local images |
| `upload_data_timeout` | `20` | The timeout [s] of the upload |
| `download_data` | `true` | Download the attachments of the received messages |
| `download_data_timeout` | `10` | Set as a parameter, but not used by the node |
| `download_directory` | `/tmp` | The directory of the downloaded attachments |
| `download_avatar` | `false` | Download the avatar images of the users into `User/avatar` |
| `use_yaml` | `false` | Load the parameters of the node from `yaml_file` too |
| `yaml_file` | `''` | ROS 1: a rosparam yaml; ROS 2: a ROS 2 parameter file (`/**: ros__parameters: ...`) |
| `respawn` | `true` | Respawn the nodes |
| `use_helper` | `true` | Start `helper.py` |
| `to_dialogflow_client` | `false` | helper: answer the messages with dialogflow_task_executive (ROS 1 only) |
| `debug_sound` | `false` | helper: say the received messages with sound_play (`robotsound_jp`) |

### google_chat_ros_node.py

#### Actions

- `~send` (`google_chat_ros/SendMessage`): send a message. Goal: `text`, `cards` (`google_chat_ros/Card[]`),
  `update_message` (update the message of the card instead of a new one), `thread_name`, `space` (must begin with
  `spaces/`). Result: `message_result` (`google_chat_ros/Message`), `done` (bool). Feedback: `status` (the error).
  Provided only when the service account key is valid.

#### Published topics

Created in the `pubsub`, `url` and `dialogflow` modes (published only in `pubsub` and `url`):

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

make sure that the subscription exists: by default a subscription expires after a period of inactivity. Set it to
never expire.

![google_chat_ros_404_error](https://user-images.githubusercontent.com/493276/230245558-f85dbde8-c774-4932-ba52-cebf6848bb5a.png)

### No `send` action

The key is missing or invalid: see the log `Failed to start Google Chat REST service` and the error after it.

<a id="notes"></a>
## 9. Notes

The Google API, Pub/Sub and gdown Python modules are installed at build time: on ROS 1 into a virtualenv by
`catkin_virtualenv` (`requirements.in`, or `requirements.in.python3.12` on Python >= 3.12), on ROS 2 into a uv venv
(`share/google_chat_ros/venv`) from `pyproject.toml`/`uv.lock` (`jsk_ros1_ros2_compat_generate_uv_venv()`). Run
`uv lock` after changing `pyproject.toml`.
