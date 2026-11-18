# gdrive_recorder

Long-term video logging with Google Drive

## Description

This packages is for logging image and audio topic in video format and uploading it to Google drive.

This package uses audio_video_recorder, influxdb and gdrive_ros.

## Sample

### Systemd

```bash
roscd gdrive_recorder
sudo cp systemd/*.service /etc/systemd/system
sudo systemctl daemon-reload
rosrun gdrive_recorder start_all_services.bash
```

### Launch

#### PR1040

```bash
rossetip
rossetmaster pr1040
roslaunch gdrive_recorder pr1040_gdrive_recorder.launch
```

#### Fetch15

```bash
rossetip
rossetmaster fetch15
roslaunch gdrive_recorder fetch15_gdrive_recorder.launch
```

#### Fetch1075

```bash
rossetip
rossetmaster fetch1075
roslaunch gdrive_recorder fetch1075_gdrive_recorder.launch
```

### Scripts

#### Systemd

```bash
# start
rosrun gdrive_recorder start_all_services.bash
# reload
rosrun gdrive_recorder reload_all_services.bash
# stop
rosrun gdrive_recorder stop_all_services.bash
```

## Node

### gdrive_recorder.py

#### Parameters

- `~robot_type`: (`String`, default: `pr2`)

  robot type

- `~robot_name`: (`String`, default: `pr2`)

  robot name

- `~timezone`: (`String`, default: `UTC`)

  timezone

- `~gdrive_server_name`: (`String`, default: `gdrive_record_server`)

  gdrive_ros upload server name

- `~upload_parents_path`: (`String`, default: `{robot_name}_recorder`)

  gdrive_ros upload parents path

- `~video_path`: (`String`, default: `/tmp`)

  temporary video saving directory

- `video_topic_name`: (`String`, default: `None`)

  video topic name. this is for checking whether video topic is available or not.

- `record_duration`: (`Int`, default: `60*20`)

  maximum recording duration into one video file

- `upload_duration`: (`Int`, default: `60*20`)

  uploading interval

- `decompress_monitor_duration`: (`Int`, default: `10`)

  monitoring decompress launch duration

- `store_url`: (`Bool`, default: `true`)

  Store gdrive_ros upload parents path in InfluxDB or not

- `host`: (`String`, default: `localhost`)

  InfluxDB host name

- `port`: (`Int`, default: `8086`)

  InfluxDB port number

- `database`: (`String`, default: `test`)

  InfluxDB  database name
