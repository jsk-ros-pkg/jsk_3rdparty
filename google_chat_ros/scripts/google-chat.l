#!/usr/bin/env roseus

(ros::load-ros-manifest "google_chat_ros")
(ros::roseus "google_chat_eus_client")

(defun send-google-chat-text (space content
                              &key (thread-name nil) (topic-name "google_chat_ros/send") (wait t))
  (wait (boundp 'google_chat_ros::SendMessageAction)
        (let ((goal (instance google_chat_ros::SendMessageActionGoal :init))
              (ac (instance ros::simple-action-client :init
                            topic-name google_chat_ros::SendMessageAction)))
          (when (send ac :wait-for-server 1)
            (when (eq (send ac :get-state) actionlib_msgs::GoalStatus::*active*)
              (send ac :cancel-goal)
              (send ac :wait-for-result :timeout 5))
            (send goal :goal :space space)
            (when thread-name
                (send goal :goal :thread_name thread-name))
            (send goal :goal :text content)
            (send ac :send-goal goal)
            (if wait
                (return-from send-google-chat-text (send ac :wait-for-result :timeout 5))
                (return-from send-google-chat-text t))))))
