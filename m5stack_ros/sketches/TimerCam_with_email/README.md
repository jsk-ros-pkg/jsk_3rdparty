# TimerCam with battery

This firmware is an addition to the [TimerCam](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros/sketches/TimerCam) firmware with email sending feature and LED feature.

- Email feature

    In the `TimerCam_with_email` sample, `/email` topic (`jsk_robot_startup/Email`) is published by `email_fridge_contents.py` once a day. This is used to report the contents of the fridge.

    To use this message, you need to clone `jsk_robot_startup` to your workspace and catkin build it.

    ```
    cd ~/m5stack_ros_ws/src/jsk-ros-pkg
    git clone https://github.com/jsk-ros-pkg/jsk_robot.git
    catkin build jsk_robot_startup
    ```

    Then, install `postfix` mail server

    ```
    sudo apt install postfix
    ```

    For more detail, see [jsk_robot_startup/email_topic.py](https://github.com/jsk-ros-pkg/jsk_robot/tree/master/jsk_robot_common/jsk_robot_startup#scriptsemail_topicpy).

- LED feature

    If you want to brighten the camera's surroundings, you can use [Grove Circular LED](https://wiki.seeedstudio.com/Grove-Circular_LED/).
    Go to [TimerCam_with_email.ino](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros/sketches/TimerCam_with_email/TimerCam_with_email.ino) and add `#define GROVE_CIRCULAR_LED`.

- Sleep feature

    In this sketch, you can use two types of sleep

      - Define BMM8563_SLEEP for low energy consumption sleep
      - Do not define BMM8563_SLEEP if you use normal delay

    You can check TimerCam sleep sample at [wakeup sample](https://github.com/m5stack/TimerCam-arduino/blob/eabb74fb56a87bc1373b1c25bc924560fa6d1e04/examples/wakeup/wakeup.ino)

Run. In addition to rosserial, an email client node and an email server node are launched.

```
roslaunch m5stack timer_cam_with_email.launch
```
