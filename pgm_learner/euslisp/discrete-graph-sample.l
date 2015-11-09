;; pgm-learner-client-sample.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pgm-learner-client "pgm-learner-client.l")

(setq *graph* (instance discrete-bayesian-graph :init
                    '((:a -> :b))))

(setq states (alist->graph-states '(((:a . :x1)
                                    (:b . 1))
                                   ((:a . :x2)
                                    (:b . 1))
                                   ((:a . :x1)
                                    (:b . 1))
                                   ((:a . :x1)
                                    (:b . 1))
                                   ((:a . :x1)
                                    (:b . 1)))))

(ros::ros-info "parameter estimation")
(dolist (n (send *graph* :parameter-estimation states))
  (print-ros-msg n))

(ros::ros-info "query")
(dolist (n (send *graph* :query '(:a) :evidence '((:b . 1))))
  (print-ros-msg n))

(provide :pgm-learner-client-sample) ;; end of pgm-learner-client-sample.l
