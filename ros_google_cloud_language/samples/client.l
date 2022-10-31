#!/usr/bin/env roseus

(ros::roseus-add-msgs "ros_google_cloud_language")
(ros::roseus "cloud_language_client")

(defun type-to-string-entity (id)
  (do-symbols (type "ROS_GOOGLE_CLOUD_LANGUAGE::TEXTENTITY")
              (if (= (eval type) id)
                  (return-from type-to-string-entity (symbol-name type))))
  nil)

(defun type-to-string-part-of-speech (id)
  (do-symbols (type "ROS_GOOGLE_CLOUD_LANGUAGE::PARTOFSPEECHTAG")
	      (if (= (eval type) id)
		  (return-from type-to-string-part-of-speech (symbol-name type))))
  nil)

(defun type-to-string-dependency-edge (id)
  (do-symbols (type "ROS_GOOGLE_CLOUD_LANGUAGE::DEPENDENCYEDGE")
	      (if (= (eval type) id)
		  (return-from type-to-string-dependency-edge (symbol-name type))))
  nil)

;; check if this retuns correct answer with https://cloud.google.com/natural-language
(defun demo ()
  (let (goal ac ret)
    (setq goal (instance ros_google_cloud_language::AnalyzeTextActionGoal :init))
    (send goal :goal :text "吾輩は猫である。名前はまだ無い。")
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
      (ros::ros-info "entiteis")
      (dolist (entity (send ret :entities))
        (ros::ros-info " - name : ~A" (send entity :name))
        (ros::ros-info "   type : ~A" (type-to-string-entity (send entity :type)))
        (ros::ros-info "   salience : ~A" (send entity :salience))
        )
      (ros::ros-info "syntaxes")
      (dolist (syntax (send ret :syntaxes))
	(ros::ros-info " - name : ~A" (send syntax :name))
	(ros::ros-info "   lemma : ~A" (send syntax :lemma))
	(ros::ros-info "   dependency_edge : ~A" (send syntax :dependency_edge))
	(ros::ros-info "   part_of_speech : ~A"
		       (type-to-string-part-of-speech (send syntax :part_of_speech)))
	(ros::ros-info "   parse_label : ~A"
		       (type-to-string-dependency-edge (send syntax :parse_label)))
	)
      )))

(ros::roseus "client")
(demo)
(exit)
