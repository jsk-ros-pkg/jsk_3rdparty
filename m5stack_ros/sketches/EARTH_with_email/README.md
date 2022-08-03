# EARTH with email

This firmware is an addition to the [EARTH](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros/sketches/EARTH) firmware with email functionality.

In the `EARTH_with_email` sample, `/email` topic (`jsk_robot_startup/Email`) is published when the `moisture` is over threshold.
This is used to report the amount of water in the spot cooler tank.
