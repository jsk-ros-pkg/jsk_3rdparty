#!/usr/bin/env roseus

(ros::roseus-add-msgs "dialogflow_task_executive")
(ros::roseus "dialogflow_example")

(defun json-to-list (str)
  (let (ret sstr)
    ;; skip first {
    (setq str (string-trim (format nil "{}~% ") str))
    ;; get first elemnet
    (while str
      (if (position #\, str)
          (setq sstr (subseq str 0 (position #\, str))
                str (subseq str (1+ (position #\, str))))
        (setq sstr str str nil))
      (push (cons (string-trim (format nil "\"~% ") (subseq sstr 0 (position #\: sstr)))
                  (string-trim (format nil "\"~% ") (subseq sstr (+ (position #\: sstr) 2)))) ret)
      )
    ret))
(defclass dialogflow-client
  :super propertied-object
  :slots (response-msg string-msg response-sub publish-tm))

(defmethod dialogflow-client
  (:init
   ()
   (setq response-sub (ros::subscribe "/dialog_response" dialogflow_task_executive::DialogResponse #'send self :dialog-cb))
   (setq response-msg (instance dialogflow_task_executive::DialogResponse :init))
   (setq string-msg (instance std_msgs::String :init))
   (ros::advertise "/text" std_msgs::string 1)
   ;; wait for text publisher
   (ros::rate 1)
   (while (< (ros::get-num-subscribers "/text") 1)
     (ros::sleep))
   self)
  (:dialog-cb
   (msg)
   (setq response-msg msg))
  (:wait-for-response
   ()
   (while (< (send (ros::time- (send response-msg :header :stamp) publish-tm) :to-sec) 0)
     (ros::ros-info "wait for response ... ")
     (ros::spin-once)
     (ros::sleep))
   response-msg)
  (:publish-dialog
   (text)
   (let (ret)
     (setq publish-tm (ros::time-now))
     (send string-msg :data text)
     (ros::publish "/text" string-msg)
     (setq ret (send self :wait-for-response))
     (send ret :parameters (json-to-list (send ret :parameters)))
     (ros::ros-info "action     : ~A" (send ret :action))
     (ros::ros-info "query      : ~A" (send ret :query))
     (ros::ros-info "response   : ~A" (send ret :response))
     (ros::ros-info "parameters : ~A" (send ret :parameters))
     (ros::ros-info "    speech : ~A" (send ret :speech_score))
     (ros::ros-info "    intent : ~A" (send ret :intent_score))
     ret))
  )


(ros::roseus "client")
(setq client (instance dialogflow-client :init))
(send client :publish-dialog "一緒に映画見よう")
(send client :publish-dialog "壁紙がきれいだったよ")
(send client :publish-dialog "一緒に飾り付けしよう")
(send client :publish-dialog "風船を右に置いて")
(exit)
