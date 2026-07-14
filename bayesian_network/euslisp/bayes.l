;; bayes.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "bayesian_network")

(ros::roseus "bayes")

(defclass cpt
  :super propertied-object
  :slots (prob))
(defmethod cpt
  (:init (node)
   (send self :name (send node :name))
   (let ((parent-num (send node :parents

(defclass bayesian-node
  :super node
  :slots (parents))
(defmethod bayesian-node
  (:init (n &optional parents)
    (send self :name n)
    (setf (bayesian-node-parents self) parents)
    self)
  (:parents (&optional v)
    (when v (setq parents v))
    parents))
  (:add-parent (p)
    (if (find p parents)
    

(defclass discrete-node
  :super node
  :slots (values cpt))
(defmethod discrete-node
  (:init (name
          &key
            ((:values ivals) nil)
            ((:cpt icpt) nil))
   (send self :name name)
   (setq values ivals)
   (setq cpt icpt)
   self)
  (:prob (&optional parents)
   (
