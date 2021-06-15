#!/usr/bin/env roseus

(ros::load-ros-manifest "switchbot_ros")
(ros::roseus "switchboteus_client")

(defun control-device (deviceName command &key (parameter "") (commandType "") (topic-name "switchbot_ros") (wait nil))
  (when (boundp 'switchbot_ros::SwitchBotCommandAction)
    (let ((goal (instance switchbot_ros::SwitchBotCommandActionGoal :init))
          (ac (instance ros::simple-action-client :init
                        topic-name switchbot_ros::SwitchBotCommandAction)))
      (when (send ac :wait-for-server 1)
        (when (eq (send ac :get-state) actionlib_msgs::GoalStatus::*active*)
          (send ac :cancel-goal)
          (send ac :wait-for-result :timeout 5))
        (send goal :goal :deviceName deviceName)
        (send goal :goal :command command)
        (send goal :goal :parameter parameter)
        (send goal :goal :commandType commandType)
        (send ac :send-goal goal)
        (if wait
            (return-from control-device (send ac :wait-for-result :timeout 5))
            (return-from control-device t))))))
