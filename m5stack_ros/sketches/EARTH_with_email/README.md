# EARTH with email

This firmware is an addition to the [EARTH](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros/sketches/EARTH) firmware with email functionality.

In the `EARTH_with_email` sample, `/email` topic (`jsk_robot_startup/Email`) is published when the `moisture` is over threshold.
This is used to report the amount of water in the spot cooler tank.

To use this message, you need to clone `jsk_robot_startup` to your workspace and catkin build it.

```
cd ~/m5stack_ros_ws/src/jsk-ros-pkg
git clone https://github.com/jsk-ros-pkg/jsk_robot.git
catkin build jsk_robot_startup
```

For more detail, see [jsk_robot_startup/email_topic.py](https://github.com/jsk-ros-pkg/jsk_robot/tree/master/jsk_robot_common/jsk_robot_startup#scriptsemail_topicpy).
