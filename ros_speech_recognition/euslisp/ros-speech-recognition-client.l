#!/usr/bin/env roseus
;; This is the euslisp version of ros_speech_recognition client.

(defclass speech-recognition-client
    :slots (speech-recog-req speech-recog-res text))
(defmethod speech-recognition-client
    (:init ()
     (ros::roseus-add-srvs "speech_recognition_msgs")
     (ros::wait-for-service "speech_recognition"))
  (:recognize ()
    (setq speech-recog-req
          (instance speech_recognition_msgs::SpeechRecognitionRequest :init))
    (setq speech-recog-res (ros::service-call "speech_recognition" speech-recog-req))
    (ros::ros-info "~A" speech-recog-req)
    (setq text (send (send speech-recog-res :result) :serialize))
    text))
