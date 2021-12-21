ros_google_cloud_language
=========================

A ROS package for google cloud natural language API
see https://cloud.google.com/natural-language for more info.


Tutorials
---------

1. Install this pacakge

```
catkin b ros_google_cloud_language
```

Do not run `pip install google-cloud-language` manually, it is installed via `catkin_virtual_env` to work with rospy.


2. Download google credentials file. For JSK users, you can download from [Google Drive](https://drive.google.com/file/d/1VxniytpH9J12ii9jphtBylydY1_k5nXf/view?usp=sharing) link.


3. Start sample code

```
roslaunch demo.launch google_cloud_credentials_json:=${HOME}/Downloads/eternal-byte-236613-4bc6962824d1.json
```
