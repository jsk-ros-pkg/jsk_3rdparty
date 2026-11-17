chaplus_ros
===========

A chatbot node: it sends the text received on `request` to a chat API on the web and publishes the reply on
`response`. Three engines are supported: [mebo](https://mebo.work/) (default), [A3RT Talk API](https://a3rt.recruit.co.jp/product/talkAPI)
and Chaplus (the Chaplus service ended on August 31, 2022; the code is kept). The same script runs on ROS 1 and ROS 2.

## Requirements

An API key of the engine, in a JSON file given by the `~chaplus_apikey_file` parameter:

```json
{"apikey_mebo": "<mebo API key>",
 "agentid_mebo": "<mebo agent ID>",
 "uid_mebo": "<any user ID of the conversation>",
 "apikey_a3rt": "<A3RT Talk API key>",
 "apikey": "<Chaplus API key>"}
```

Only the keys of the engine in use are read.

- mebo: create an account and an agent at https://mebo.work/ and take the API key and the agent ID from the API
  settings of the agent (see also https://qiita.com/maKunugi/items/14f1b82a2c0b6fa5c202). `uid_mebo` is a user ID of
  your choice; mebo keeps the context of the conversation per user ID.
- A3RT: register at https://a3rt.recruit.co.jp/product/talkAPI/registered/ and the API key is sent by e-mail.
- JSK members can download a ready `apikey.json` from
  [Google Drive](https://drive.google.com/file/d/1tAT_WQqCMqvtbM0-CSTomWjwMP4jcOi9/view?usp=sharing).

The node needs access to the internet (the endpoints are fixed in the code).

## Usage

### ROS 2

There is no ROS 2 launch file; run the node:

```bash
ros2 run chaplus_ros chaplus_ros.py --ros-args \
  -p chatbot_engine:=Mebo -p chaplus_apikey_file:=$HOME/Downloads/apikey.json -p use_sample:=false
```

Talk to it:

```bash
ros2 topic echo /response &
ros2 topic pub --once /request std_msgs/msg/String "{data: 'おはよう'}"
```

### ROS 1

```bash
rosrun chaplus_ros chaplus_ros.py _chatbot_engine:=Mebo _chaplus_apikey_file:=$HOME/Downloads/apikey.json _use_sample:=false
rostopic echo /response &
rostopic pub -1 /request std_msgs/String "data: 'おはよう'"
```

`sample/google_example.launch` (ROS 1 only) connects it to speech recognition and speech synthesis:

```bash
roslaunch chaplus_ros google_example.launch chaplus_apikey_file:=$HOME/Downloads/apikey.json
```

| Argument | Default | Description |
|---|---|---|
| `chatbot_engine` | `Mebo` | `Mebo`, `A3RT` or `Chaplus` |
| `use_sample` | `false` | Send `communication_sample.json` with each request (Chaplus only) |
| `use_respeaker` | `true` | `true`: relay `/speech_to_text` (`speech_recognition_msgs/SpeechRecognitionCandidates`, e.g. of respeaker_ros) to `/request`. `false`: start `aques_talk` and `ros_speech_recognition` (GoogleCloud engine) and relay `/Tablet/voice` to `/request` |
| `chaplus_apikey_file` | `$(find chaplus_ros)/apikey.json` | The API key file |

The reply on `/response` is relayed to `/robotsound_jp` (`sound_play/SoundRequest`). With `use_respeaker:=false`,
edit the `/speech_recognition/google_cloud_credentials_json` parameter in the launch file to your Google Cloud key
(see [ros_speech_recognition](../ros_speech_recognition)).

Try a conversation with `rostopic`:

```bash
# terminal 1
rostopic pub -1 /speech_to_text speech_recognition_msgs/SpeechRecognitionCandidates "transcript: ['おはよう']
confidence: [0]"
rostopic pub -1 /speech_to_text speech_recognition_msgs/SpeechRecognitionCandidates "transcript: ['何色が好き？']
confidence: [0]"
# terminal 2
rostopic echo --filter "print(m.arg)" /robotsound_jp
おはようございます！
私の好きな色は緑です。気持ちが落ち着きますよね。
```

## Interface

### chaplus_ros.py

#### Subscribed topics

- `request` (`std_msgs/String`): the utterance to the chatbot.

#### Published topics

- `response` (`std_msgs/String`): the reply of the chatbot; `ごめんなさい、よくわからないです` when the request fails.

#### Parameters

- `~chatbot_engine` (string, default: `Mebo`): `Mebo`, `A3RT` or `Chaplus`.
- `~chaplus_apikey_file` (string, default: `apikey.json` of the package: the source directory on ROS 1, the share
  directory on ROS 2): the JSON file of the API keys.
- `~use_sample` (bool, default: `true`): load `~communication_sample_file` and send its `agentState` and `addition`
  with each request. Only the Chaplus engine uses it, but the file is loaded with any engine when this is `true`.
- `~communication_sample_file` (string, default: `communication_sample.json` of the package): the character and
  fixed utterance/response pairs of the bot (Chaplus).
