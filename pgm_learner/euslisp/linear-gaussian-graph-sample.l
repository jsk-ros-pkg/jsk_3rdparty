;; linear-gaussian-graph-sample.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "action_state_inference")
(ros::load-ros-manifest "pgm_learner")
(require :pgm-learner-client "package://pgm_learner/euslisp/pgm-learner-client.l")

(setq *data* nil)

#|
      a
     / \
    b  |
     \ /
      c
      |
      d
|#

(dotimes (i 8000)
  (let* ((a (random-gauss 20.0 18.0))
         (b (random-gauss (+ 12.0 (* 3 a)) 25.0))
         (c (random-gauss (+ -4.0 (* 5 a) (* 9 b)) 100))
         (d (random-gauss (+ 100 (* 3 c)) 20)))
    (push
     (list
      (cons :a a)
      (cons :b b)
      (cons :c c)
      (cons :d d))
      *data*)))
(setq *data* (alist->graph-states *data* :graph-type :lg))

(setq *graph* (instance lg-bayesian-graph :init))

(ros::ros-info "structure estimation")
(setq structure (send *graph* :structure-estimation *data*))
(print-ros-msg structure)

(ros::ros-info "parameter estimation using estimated graph structure")
(setq params (send *graph* :parameter-estimation *data*))
(mapcar #'print-ros-msg params)

(ros::ros-info "write graph to pdf")
(send *graph* :write-to-pdf)

(provide :linear-gaussian-graph-sample) ;; end of linear-gaussian-graph-sample.l
