# gdrive_ros

Google Drive file uploader for ROS: a server node which uploads local files to Google Drive (with
[PyDrive](https://pythonhosted.org/PyDrive/)) on a service call and returns their URLs, and Python / EusLisp clients.
The same script runs on ROS 1 and ROS 2.

## Requirements

- A Google account and an OAuth 2.0 client of the Google Drive API: follow steps 1-5 of the
  [PyDrive quickstart](https://pythonhosted.org/PyDrive/quickstart.html#authentication) (create a project in the
  Google Cloud console, enable the Google Drive API, create an OAuth client ID and download it as
  `client_secrets.json`).
- A PyDrive settings yaml ([format](https://pythonhosted.org/PyDrive/oauth.html#automatic-and-custom-authentication-with-settings-yaml)),
  for example:

  ```yaml
  client_config_file: /your/client/secrets/json/path
  save_credentials: True
  save_credentials_backend: file
  save_credentials_file: /your/credentials/json/path
  get_refresh_token: True
  oauth_scope:
    - https://www.googleapis.com/auth/drive.file
  ```

  A relative `client_config_file` / `save_credentials_file` which is not found is looked up relative to the directory
  of the settings yaml.
- At the first start the node runs PyDrive's `LocalWebserverAuth()`: it opens a web browser to log in to the Google
  account, so run it once on a machine with a browser. With `save_credentials: True` the token is saved to
  `save_credentials_file` and later starts need no login.
- Python dependencies: PyDrive is not available from apt. On ROS 1 it is installed into a virtualenv by
  `catkin_virtualenv` (`requirements.txt`); on ROS 2 into a uv venv (`share/gdrive_ros/venv`) from
  `pyproject.toml`/`uv.lock` at build time (`jsk_ros1_ros2_compat_generate_uv_venv()`). Run `uv lock` after changing
  `pyproject.toml`.

## Usage

### ROS 2

```bash
ros2 launch gdrive_ros gdrive_server.launch.xml settings_yaml:=/your/settings/yaml/path
```

```bash
ros2 service call /gdrive_server/upload gdrive_ros/srv/Upload \
  "{file_path: /tmp/test.txt, parents_path: example_folder}"
ros2 service call /gdrive_server/upload_multi gdrive_ros/srv/MultipleUpload \
  "{file_paths: [/tmp/a.txt, /tmp/b.txt], file_titles: [a.txt, b.txt], use_timestamp_folder: true}"
```

The sample client uploads one file with both services:

```bash
ros2 run gdrive_ros sample_gdrive_rospy_client.py --ros-args \
  -p file_name:=/tmp/test.txt -p file_title:=test.txt -p parents_path:=/example_folder
```

### ROS 1

```bash
export GOOGLE_DRIVE_SETTINGS_YAML=/your/settings/yaml/path
roslaunch gdrive_ros gdrive_server.launch
rosservice call /gdrive_server/upload "{file_path: /tmp/test.txt, parents_path: example_folder}"
```

Samples, which start the server and upload a file with the Python or the EusLisp client:

```bash
roslaunch gdrive_ros sample_rospy_client.launch file_name:=/tmp/test.txt file_title:=test.txt
roslaunch gdrive_ros sample_roseus_client.launch file_name:=/tmp/test.txt file_title:=test.txt
```

### Launch arguments

`launch/gdrive_server.launch` (ROS 1) and `launch/gdrive_server.launch.xml` (ROS 2):

| Argument | Default | Description |
|---|---|---|
| `settings_yaml` | `$GOOGLE_DRIVE_SETTINGS_YAML`, or `/var/lib/robot/pydrive_settings.yaml` | The PyDrive settings yaml |
| `respawn` | `false` | Respawn the node |
| `node_name` | `gdrive_server` | The node name (and so the namespace of the services) |
| `machine` | `localhost` | ROS 1 only: the machine to run the node |

### Clients

- Python: `gdrive_ros.gdrive_ros_client.GDriveROSClient(node)` with `wait_for_gdrive_server()`, `upload_file()` and
  `upload_multiple_files()` (ROS 1 and ROS 2; see `node_scripts/sample_gdrive_rospy_client.py`).
- EusLisp (ROS 1): `euslisp/gdrive-ros-utils.l` with `wait-for-gdrive-server`, `upload-file` and
  `upload-multiple-files` (see `euslisp/sample-gdrive-roseus-client.l`).

Both call `/gdrive_server/upload` and `/gdrive_server/upload_multi`, i.e. the server with the default `node_name`.

## Interface

### gdrive_server_node.py

#### Services

- `~upload` (`gdrive_ros/Upload`): upload a file.
- `~upload_multi` (`gdrive_ros/MultipleUpload`): upload files into one folder.

The services are advertised after the authentication has finished.

#### Parameters

- `~settings_yaml` (string, required): the PyDrive settings yaml.
- `~share_type` (string, default: `anyone`): the [permission type](https://developers.google.com/drive/api/v2/reference/permissions)
  of the uploaded files: `user`, `group`, `domain` or `anyone`.
- `~share_value` (string, default: `anyone`): the e-mail address or domain of the permission.
- `~share_role` (string, default: `reader`): the role of the permission.
- `~share_with_link` (bool, default: `true`): share with a link.
- `~auth_max_trial` (int, default: `-1`): the number of trials of the authentication when the server is not found
  (no network); `-1` is infinite.
- `~auth_wait_seconds` (float, default: `10.0`): the wait between the trials.

### gdrive_ros/Upload

Request:

- `file_path` (string): the local file; `~` is expanded.
- `file_title` (string): the title in Google Drive; empty means the base name of `file_path`.
- `parents_path` (string): the folder in Google Drive, e.g. `a/b`; the folders are created when not found. Empty
  means the root.
- `parents_id` (string): the ID of the folder; used instead of `parents_path` when both are set.
- `use_timestamp_folder` (bool): upload into a new folder named by the timestamp (`%Y%m%d%H%M%S`) in the folder.
- `use_timestamp_file_title` (bool): prefix the title with the timestamp, as `<timestamp>_<file_title>`.

Response: `success` (bool), `file_id` (string), `file_url` (string, a download URL), `parents_id` (string) and
`parents_url` (string, the URL of the folder).

### gdrive_ros/MultipleUpload

The same as `Upload` with `file_paths` (string[]) and `file_titles` (string[]) for the files, and `successes`
(bool[]), `file_ids` (string[]) and `file_urls` (string[]) in the response. Give `file_titles` the same length as
`file_paths` (an empty string uses the base name); only as many files as `file_titles` are uploaded.

## Troubleshooting

On Ubuntu 18.04 with Python 2.7, `rsa < 4.6.0` is required:

```bash
pip install oauth2client==4.1.3 rsa==4.5 pydrive==1.3.1
```
