#!/usr/bin/env roseus

(require "package://gdrive_ros/euslisp/gdrive-ros-utils.l")

(ros::roseus "sample_gdrive_ros_client")

(wait-for-gdrive-server)
(ros::ros-info "Gdrive server ready.")

(setq file-name (ros::get-param "~file_name"))
(setq file-title (ros::get-param "~file_title"))
(setq parents-path (ros::get-param "~parents_path"))

(ros::ros-info "Uploading files...")
(setq res (upload-file file-name file-title :parents-path parents-path))
(if res
    (ros::ros-info "File uploading succeeded. res: ~A" res)
  (ros::ros-error "File uploading failed."))

(ros::ros-info "Uploading multiple files...")
(setq res (upload-multiple-files (list file-name) (list file-title) :parents-path parents-path))
(if (elt (elt res 0) 0)
    (ros::ros-info "File uploading succeeded. res: ~A" res)
  (ros::ros-error "File uploading failed."))

(ros::roseus "shutdown")
(exit)
