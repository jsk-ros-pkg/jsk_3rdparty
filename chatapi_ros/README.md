# chatapi_ros
The purpose of this repository is to make existing chat APIs usable with ROS.

## Setting up environment
Please install following packages.
```
sudo pip install google-api-python-client
sudo pip install oauth2client
sudo pip install pya3rt
```

Please install ros_speech_recognition.
```
sudo apt install ros-${ROS_DISTRO}-ros-speech-recognition
```
%You can get the latest version from [here](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/ros_speech_recognition). Please try the latest version if it does not work well.

## Set API Keys for [Chaplus](http://www.chaplus.jp/api) and [A3RT](https://a3rt.recruit-tech.co.jp/product/talkAPI)
### Get Chaplus APIKey
Please access [here](http://www.chaplus.jp/api#contact) and enter your information.

### Get A3RT APIKey
Please access [here](https://a3rt.recruit-tech.co.jp/product/talkAPI/registered/) and enter your information.

### Write your keys in `settings.yaml`
```
chaplusAPIKEY: xxxxxxxxxx
a3rtAPIKEY: yyyyyyyyyy
```

## Get json file for Google speech recognition API.
### Get json file
Please follow the step 1 and 2 in [here](https://cloud.google.com/speech-to-text/docs/quickstart-protocol) and download json file.
### Place the json file in this repository.
`chatapi_ros/your-json-name.json`


## Rewrite `launch/chatapi.launch` file
### Change the path of json file.
```
<param name="/speech_recognition/google_cloud_credentials_json"
       value="$(find chatapi_ros)/your-json-name.json" />
```

### Change the microphone settings.
Check your microphone.
```
pactl list short sinks
```
Rewrite the following settings according to your environment. 
```
<arg name="n_channel" default="2" />
<arg name="depth" default="16" />
<arg name="sample_rate" default="44100" />
```

Check your card and device number of microphone.
```
arecord -l
```
Rewrite the following settings to "hw:[card number],[device number]" . 
```
<arg name="device" default="hw:0,0" />
```
 
### Change other settings
Change the launguage you want to use.
```
<arg name="language" default="en" />
```
If you do not want to use Chaplus
```
<arg name="use_chaplus" default="false" />
```
If you do not want to use A3RT
```
<arg name="use_a3rt" default="false" />
```

## Build
```
catkin build chatapi_ros
source devel/setup.bash
```

## Usage
```
roslaunch chatapi_ros chatapi.launch
```
You can access the result in rostopics. (`/chatAPI/a3rt` and `/chatAPI/chaplus`)