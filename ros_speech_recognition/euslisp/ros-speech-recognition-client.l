#!/usr/bin/env roseus
;; This is the euslisp version of ros_speech_recognition client.

(ros::load-ros-manifest "speech_recognition_msgs")
(ros::roseus-add-srvs "speech_recognition_msgs")


(defclass speech-recognition-client
    :slots (speech-recog-req speech-recog-res text))
(defmethod speech-recognition-client
  (:init ()
    (ros::wait-for-service "speech_recognition"))
  (:recognize (&rest args)
    (setq speech-recog-req
          (instance speech_recognition_msgs::SpeechRecognitionRequest :init))
    (send speech-recog-req :duration 1)
    (send speech-recog-req :quiet t)
    (setq speech-recog-res (ros::service-call "speech_recognition" speech-recog-req))
    (ros::ros-info "~A" speech-recog-req)
    (setq text (send (send speech-recog-res :result) :transcript))
    (unless (null text)
      (ros::ros-info "~A" (elt text 0)))
    text))
