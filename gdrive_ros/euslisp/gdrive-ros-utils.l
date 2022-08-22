(ros::load-ros-manifest "gdrive_ros")

(defun wait-for-gdrive-server (&rest args)
  (if args
      (ros::wait-for-service "/gdrive_server/upload" args)
    (ros::wait-for-service "/gdrive_server/upload")
    )
  )

(defun upload-file (file-path
                    file-title
                    &key
                    (parents-path nil)
                    (parents-id nil)
                    (use-timestamp-folder nil)
                    (use-timestamp-file-title nil))
  (let ((req (instance gdrive_ros::UploadRequest :init))
        (res nil))
    (send req :file_path file-path)
    (send req :file_title file-title)
    (send req :parents_path parents-path)
    (send req :parents_id parents-id)
    (send req :use_timestamp_folder use-timestamp-folder)
    (send req :use_timestamp_file_title use-timestamp-file-title)
    (setq res (ros::service-call "/gdrive_server/upload" req t))
    (list (send res :success)
          (send res :file_id)
          (send res :file_url)
          (send res :parents_id)
          (send res :parents_url))
    ))

(defun upload-multiple-files
                   (file-paths
                    file-titles
                    &key
                    (parents-path nil)
                    (parents-id nil)
                    (use-timestamp-folder nil)
                    (use-timestamp-file-title nil))
  (let ((req (instance gdrive_ros::MultipleUploadRequest :init))
        (res nil))
    (send req :file_paths file-paths)
    (send req :file_titles file-titles)
    (send req :parents_path parents-path)
    (send req :parents_id parents-id)
    (send req :use_timestamp_folder use-timestamp-folder)
    (send req :use_timestamp_file_title use-timestamp-file-title)
    (setq res (ros::service-call "/gdrive_server/upload_multi" req t))
    (list (send res :successes)
          (send res :file_ids)
          (send res :file_urls)
          (send res :parents_id)
          (send res :parents_url))
    ))
