# gdrive_ros

Google Drive file uploader for ROS

## Installation

### Setup and build workspace

```bash
cd ~
mkdir gdrive_ws/src -p
cd gdrive_ws/src
git clone https://github.com/jsk-ros-pkg/jsk_3rdparty.git
rosdep install --ignore-src -from-paths . -y -r
cd ~/gdrive_ws
catkin init
catkin build
```

### Trouble shooting

If you use 18.04 and python2.7, please install following versions.
`
```bash
pip install oauth2client==4.1.3 rsa==4.5 pydrive==1.3.1
```

### Do authentication of Google Drive API

Please follow step 1-5 in [here](https://pythonhosted.org/PyDrive/quickstart.html#quickstart).

### Create your settings yaml

Please follow [here](https://pythonhosted.org/PyDrive/oauth.html#automatic-and-custom-authentication-with-settings-yaml).

My settings yaml is as follows;

```yaml
client_config_file: /your/client/secrets/json/path
save_credentials: True
save_credentials_backend: file
save_credentials_file: /your/credentials/json/path
get_refresh_token: True
oauth_scope:
  - https://www.googleapis.com/auth/drive.file
```

If you set `save_credentials: True`, you need to login to your Google account only for the first time.

### Run server for the first time and login to Google account

Run google drive server and login to Google account for the first time.

```bash
export GOOGLE_DRIVE_SETTINGS_YAML=/your/settings/yaml/path
roslaunch gdrive_ros gdrive_server.launch
```


## Usage

### Run server
```bash
export GOOGLE_DRIVE_SETTINGS_YAML=/your/settings/yaml/path
roslaunch gdrive_ros gdrive_server.launch
```

### Call upload service

```bash
# single upload
rosservice call /gdrive_ros/upload ...
# multiple upload
rosservice call /gdrive_ros/upload_multi ...
```

## Parameters

- `~settings_yaml` (`string`, default: `None`)

  - PyDrive settings yaml path

- `~share_type` (`string`, default: `anyone`, candidates: `user`, `group`, `domain` and `anyone`)

  - Uploaded file share type

  - For more information, please read [here](https://developers.google.com/drive/api/v3/reference/permissions#type).

- `~share_value` (`string`, default: `anyone`)

  - Uploaded file share value
  
- `~share_role` (`string`, default: `reader`)

  - Uploaded file share role

- `~share_with_link` (`bool`, default: `true`)

  - Uploaded file share with link or not 

- `~auth_max_trial` (`int`, default: `-1`)

  - Max authentication trial times. `-1` means trying for infinite times.

- `~auth_wait_seconds` (`float`, default: `10.0`)

  - Authentication wait seconds

## Services

### `gdrive_ros/Upload`

This service is for uploading single file in same Google Drive folder.

**Request**

- `file_path` (`string`, default: `''`)

  - Uploaded file path

- `file_title` (`string`: default: `file_path.split('/')[-1]`)

  - Uploaded file title in Google Drive


- `parents_path` (`string`, default: `''`)

  - Parents path in Google Drive splitted by `/`


- `parents_id` (`string`, default: `''`)

  - Parents id in Google Drive

  - If both `parents_path` and `parents_id` are set , `parents_id` will be used.

- `use_timestamp_folder` (`bool`, default: `false`)

  - Use timestamp folder to upload

  - Uploaded file will be saved in `file_path/timestamp` folder.

- `use_timestamp_file_title` (`bool`, default: `false`)

  - Use timestamp for `file_title`

  - Uploaded file will be save as `{}_{}.format(timestamp file_title)`. 


**Response**

- `success` (`bool`)

  - Upload succeeded or not

- `file_id` (`string`)

  - Uploaded file id in Google Drive

- `file_url` (`string`)

  - Uploaded file url in Google Drive

- `parents_id` (`string`)

  - Parents folder id of uploaded file in Google Drive

- `parents_url` (`string`)

  - Parents folder url of uploaded file in Google Drive

### `gdrive_ros/MultipleUpload`

This service is for uploading multiple files in same Google Drive folder.

**Request**

- `file_paths` (`string[]`, default: `[]`)

  - Uploaded file paths

- `file_titles` (`string[]`: default: `[f for f in file_paths.split('/')[-1]]`)

  - Uploaded file titles in Google Drive


- `parents_path` (`string`, default: `''`)

  - Parents path in Google Drive splitted by `/`


- `parents_id` (`string`, default: `''`)

  - Parents id in Google Drive

  - If both `parents_path` and `parents_id` are set , `parents_id` will be used.

- `use_timestamp_folder` (`bool`, default: `false`)

  - Use timestamp folder to upload

  - Uploaded file will be saved in `file_path/timestamp` folder.

- `use_timestamp_file_title` (`bool`, default: `false`)

  - Use timestamp for `file_title`

  - Uploaded file will be save as `{}_{}.format(timestamp file_title)`. 


**Response**

- `successes` (`bool[]`)

  - Upload succeeded or not

- `file_ids` (`string[]`)

  - Uploaded file ids in Google Drive

- `file_urls` (`string[]`)

  - Uploaded file urls in Google Drive

- `parents_id` (`string`)

  - Parents folder id of uploaded file in Google Drive

- `parents_url` (`string`)

  - Parents folder url of uploaded file in Google Drive
