;; bayesian-network.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "bayesian_network")

(unless (find-package "BAYES")
  (make-package "BAYES"))
(in-package "BAYES")

(defun keyword->name-string (k)
  (substitute #\- #\_
    (string-downcase (string k))))

(defclass cpts
  :super propertied-object
  :slots (table values parents))
(defmethod cpts
  (:init (n values &optional parents)
   (send self :name n)
   (setf (cpts-values self) values)
   (setf (cpts-parents self) parents)
   (send self :init-table)
   self)
  (:init-table ()
   (setq table
         (make-matrix
          (apply #'* (mapcar #'(lambda (p)
                                 (length (send p :values)))
                             parents))
          (length values))))
  (:add-to-table (prob &optional states)
    (dolist (p parents)
     (let* ((pcons (assoc (send p :name) states))
            (idx 0))
       (


(defclass discrete-node
  :super node
  :slots (cpts)) ;; '((("node_a" "A" "node_b" "C")) . 0.5) ((()
(defmethod discrete-node
  (:init (n cpts)
   (send self :name n)
   (send self :cpts cpts)
   self)
  (:cpts (&optional value)
   (when value (setq cpts value))
   cpts)
  (:prob (&optional conditions)
                1)
  (

(defun eus-discrete-graph->ros-msg (gr)
  (


(provide :bayesian-network) ;; end of bayesian-network.l
