#!/usr/bin/env roseus

(ros::roseus-add-msgs "ros_google_cloud_language")
(ros::load-ros-package "std_msgs")
(setq *sub-topic* "/response")


(defun type-to-string (id)
  (do-symbols (type "ROS_GOOGLE_CLOUD_LANGUAGE::TEXTENTITY")
              (if (= (eval type) id)
                  (return-from type-to-string (symbol-name type))))
  nil)

;; check if this retuns correct answer with https://cloud.google.com/natural-language
(defun send-text (text)
  (let (goal ac ret)
    (setq goal (instance ros_google_cloud_language::AnalyzeTextActionGoal :init))
    (send goal :goal :text text)
    (ros::ros-info "analyze : ~A" (send goal :goal :text))
    ;;
    (setq ac (instance ros::simple-action-client :init
                   "/analyze_text/text" ros_google_cloud_language::AnalyzeTextAction))
    (send ac :wait-for-server)
    ;;
    (send ac :send-goal goal)
    (when (send ac :wait-for-result :timeout 5)
      (setq ret (send ac :get-result))
      (ros::ros-info "sentiment")
      (ros::ros-info " score : ~A" (send ret :sentiment_score))
      (ros::ros-info " magnitude : ~A" (send ret :sentiment_magnitude))
      (ros::ros-info "entities")
      (dolist (entity (send ret :entities))
        (ros::ros-info " name : ~A" (send entity :name))
        (ros::ros-info " type : ~A" (type-to-string (send entity :type)))
        (ros::ros-info " salience : ~A" (send entity :salience))
        )
      )))

(defun chat-cb (msg)
  (setq text (send msg :data))
  (send-text text)
  )

(defun chat-demo ()
  (ros::subscribe *sub-topic* std_msgs::String #'chat-cb)
  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep))
  )

(ros::roseus "client")
(chat-demo)
(exit)
