;; monty-hall.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "bayesian_network")
(ros::roseus "monty_hall_ros")

(setq values (list "A" "B" "C"))
(setq nodes
      (list
       (instance bayesian_network::DiscreteNode :init
                 :name "prize_door"
                 :parents nil
                 :children '("monty_door")
                 :values 


(provide :monty-hall) ;; end of monty-hall.l
