#!/usr/bin/env roseus

(ros::load-ros-manifest "switchbot_ros")
(ros::roseus "switchboteus_client")

(defun get-devices (&key (topic-name "switchbot_ros/devices"))
  (let (msg)
    (setq msg (one-shot-subscribe topic-name switchbot_ros::DeviceArray))
    msg))

(defun control-device (device_name command &key (parameter "") (command_type "") (topic-name "switchbot_ros/switch") (wait nil))
  "
  Please see the following page for command details.
  https://github.com/OpenWonderLabs/SwitchBotAPI#command-set-for-physical-devices
  "
  (when (boundp 'switchbot_ros::SwitchBotCommandAction)
    (let ((goal (instance switchbot_ros::SwitchBotCommandActionGoal :init))
          (ac (instance ros::simple-action-client :init
                        topic-name switchbot_ros::SwitchBotCommandAction)))
      (when (send ac :wait-for-server 1)
        (when (eq (send ac :get-state) actionlib_msgs::GoalStatus::*active*)
          (send ac :cancel-goal)
          (send ac :wait-for-result :timeout 5))
        (send goal :goal :device_name device_name)
        (send goal :goal :command command)
        (send goal :goal :parameter parameter)
        (send goal :goal :command_type command_type)
        (send ac :send-goal goal)
        (if wait
            (return-from control-device (send ac :wait-for-result :timeout 5))
            (return-from control-device t))))))
