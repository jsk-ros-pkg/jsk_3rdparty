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
roslaunch ros_google_cloud_language demo.launch google_cloud_credentials_json:=${HOME}/Downloads/eternal-byte-236613-4bc6962824d1.json
```

Additional Topic: Chat with agent
---------

If you have already run the tutorials, please skip the STEP1 and 2.

1. Install this pacakge

```
catkin b ros_google_cloud_language
```

Do not install `google-cloud-language` manually, it is installed via `catkin_virtual_env` to work with rospy.


2. Download google credentials file. For JSK users, you can download from [Google Drive](https://drive.google.com/file/d/1VxniytpH9J12ii9jphtBylydY1_k5nXf/view?usp=sharing) link.


3. Install Chaplus package

```
catkin build chaplus_ros
```

for more detail, please show [chaplus_ros repository](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/chaplus_ros)

4. Download apikey. For JSK users, you can download from [Google Drive](https://drive.google.com/file/d/1wh1_WX3l_qKbUG5wdgeQQBQCu6f9BSWF/view?usp=sharing) link, and please replace `chaplus_ros/apikey.json` with this.

5. Run the sample code

```
roslaunch ros_google_cloud_language demo_with_chat.launch google_cloud_credentials_json:=${HOME}/Downloads/eternal-byte-236613-4bc6962824d1.json
```

6. Launch another terminal, and please send what you want to tell the agent, like the following example

```
rostopic pub -1 /request std_msgs/String "data: 'おはよう'"
```
