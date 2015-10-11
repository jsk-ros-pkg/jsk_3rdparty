;; linear-gaussian-graph-sample.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "action_state_inference")
(ros::load-ros-manifest "pgm_learner")
(require :pgm-learner-client "package://pgm_learner/euslisp/pgm-learner-client.l")

(setq *results* nil)

(setq *mu-a* 200.0)
(setq *mu-c* 30.0)

(dotimes (i 8000)
  (let ((a-val (random-gauss *mu-a* 18.0)))
    (push
     (list
      (cons :a a-val)
      (cons :b (random-gauss (+ 12.0 (* 0.3 a-val)) 25.0)) ;; :b depends :a
      (cons :c (random-gauss *mu-c* 3.0)))
     *results*)))

(nreverse *results*)
(setq *graph* (instance lg-bayesian-graph :init))
(setq res (send *graph* :structure-estimation
                (alist->graph-states *results* :graph-type :lg)))
(print-ros-msg res)

(provide :linear-gaussian-graph-sample) ;; end of linear-gaussian-graph-sample.l
