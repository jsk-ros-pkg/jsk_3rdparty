#!/usr/bin/env roseus

(ros::load-ros-manifest "google_chat_ros")
(ros::roseus "google_chat_eus_client")

(defun send-google-chat-text (space content
                              &key (thread-name nil) (action-goal-name "google_chat_ros/send") (wait t)
                                (ac (instance ros::simple-action-client :init
                                              action-goal-name google_chat_ros::SendMessageAction)))
  (wait (boundp 'google_chat_ros::SendMessageAction)
        (let ((goal (instance google_chat_ros::SendMessageActionGoal :init)))
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
                (return-from send-google-chat-text (if (send ac :wait-for-result :timeout 5) (send ac :get-result) nil))
                (return-from send-google-chat-text t))))))

(defun send-google-chat-image
    (space image-path
     &key (image-header "") (thread-name nil)
       (action-goal-name "google_chat_ros/send") (wait t)
       (ac (instance ros::simple-action-client :init
                     action-goal-name google_chat_ros::SendMessageAction)))
  (when (boundp 'google_chat_ros::SendMessageAction)
    (let ((goal (instance google_chat_ros::SendMessageActionGoal :init)))
          (card (instance google_chat_ros::Card :init))
          (section (instance google_chat_ros::Section :init))
          (widget (instance google_chat_ros::WidgetMarkup :init))
          (image (instance google_chat_ros::Image :init)))
      (when (send ac :wait-for-server 1)
        (when (eq (send ac :get-state) actionlib_msgs::GoalStatus::*active*)
          (send ac :cancel-goal)
          (send ac :wait-for-result :timeout 5))
        (send image :localpath image-path)
        (send widget :image image)
        (send section :widgets (list widget))
        (send section :header image-header)
        (send card :sections (list section))
        (send goal :goal :cards (list card))
        (send goal :goal :space space)
        (when thread-name
          (send goal :goal :thread_name thread-name))
        (send ac :send-goal goal)
        (if wait
            (return-from send-google-chat-image (send ac :wait-for-result :timeout 5))
            (return-from send-google-chat-image t))))))

(defun create-google-chat-button
    (button-name button-action-name
     &key (button-action-key) (button-action-value))
  (let ((button (instance google_chat_ros::Button :init))
        (text-button-on-click (instance google_chat_ros::OnClick :init))
        (action (instance google_chat_ros::FormAction :init))
        (parameter (instance google_chat_ros::ActionParameter :init)))
    (send button :text_button_name button-name)
    (send action :action_method_name button-action-name)
    (send parameter :key button-action-key)
    (send parameter :value button-action-value)
    (send action :parameters (list parameter))
    (send text-button-on-click :action action)
    (send button :text_button_on_click text-button-on-click)
    button))

(defun send-google-chat-buttons
    ;; buttons should be list
    (space buttons
     &key (buttons-header "") (thread-name nil)
       (action-goal-name "google_chat_ros/send") (wait nil)
       (ac (instance ros::simple-action-client :init
                     action-goal-name google_chat_ros::SendMessageAction)))
  (when (boundp 'google_chat_ros::SendMessageAction)
    (let ((goal (instance google_chat_ros::SendMessageActionGoal :init))
          (card (instance google_chat_ros::Card :init))
          (section (instance google_chat_ros::Section :init))
          (widget (instance google_chat_ros::WidgetMarkup :init)))
      (when (send ac :wait-for-server 1)
        (when (eq (send ac :get-state) actionlib_msgs::GoalStatus::*active*)
          (send ac :cancel-goal)
          (send ac :wait-for-result :timeout 5))
        (send widget :buttons buttons)
        (send section :widgets (list widget))
        (send section :header buttons-header)
        (send card :sections (list section))
        (send goal :goal :cards (list card))
        (send goal :goal :space space)
        (when thread-name
          (send goal :goal :thread_name thread-name))
        (send ac :send-goal goal)
        (if wait
            (return-from send-google-chat-buttons (send ac :wait-for-result :timeout 5))
            (return-from send-google-chat-buttons t))))))
