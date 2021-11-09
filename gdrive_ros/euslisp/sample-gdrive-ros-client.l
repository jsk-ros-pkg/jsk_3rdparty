#!/usr/bin/env roseus

(require "package://gdrive_ros/euslisp/gdrive-ros-utils.l")

(ros::roseus "sample_gdrive_ros_client")

(wait-for-gdrive-server)
(ros::log-info "Gdrive server ready.")

(setq file-name (ros::get-param "~file_name"))
(setq file-title (ros::get-param "~file_title"))
(setq parents-id (ros::get-param "~parents_id"))

(ros::log-info "Uploading files...")
(setq res (upload-file file-name file-title :parents-id parents-id))
(if (elt res 0)
    (ros::log-info "File uploading succeeded.")
  (ros::log-error "File uploading failed."))

(ros::log-info "Uploading multiple files...")
(setq res (upload-multiple-files (list file-name) (list file-title) :parents-id parents-id))
(if (elt (elt res 0) 0)
    (ros::log-info "File uploading succeeded.")
  (ros::log-error "File uploading failed."))
