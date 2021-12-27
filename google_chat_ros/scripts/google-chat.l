#!/usr/bin/env roseus

(ros::load-ros-manifest "google_chat_ros")
(ros::roseus "google_chat_eus_client")

(defun send-google-chat-message (space content &key (message-type "text") (topic-name "google_chat_ros/send") (wait nil))
  (when (boundp 'google_chat_ros::SendMessageAction)
    (let ((goal (instance google_chat_ros::SendMessageActionGoal :init))
          (ac (instance ros::simple-action-client :init
                        topic-name google_chat_ros::SendMessageAction)))
      (when (send ac :wait-for-server 1)
        (when (eq (send ac :get-state) actionlib_msgs::GoalStatus::*active*)
          (send ac :cancel-goal)
          (send ac :wait-for-result :timeout 5))
        (send goal :goal :space space)
        (send goal :goal :message_type message-type)
        (send goal :goal :content content)
        (send ac :send-goal goal)
        (if wait
            (return-from send-google-chat-message (send ac :wait-for-result :timeout 5))
            (return-from send-google-chat-message t)))))
)
