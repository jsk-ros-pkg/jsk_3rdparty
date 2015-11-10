;; pgm-learner-client.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "pgm_learner")

(ros::roseus "pgm_learner_client")

(defun key->string (k)
  (string-downcase (string k)))

(defun alist->node-states (alst &key (graph-type :discrete))
  (case graph-type
    (:discrete
     (mapcar #'(lambda (c)
                 (instance pgm_learner::DiscreteNodeState :init
                           :node (key->string (car c))
                           :state (key->string (cdr c))))
             alst))
    (:lg
     (mapcar #'(lambda (c)
                 (instance pgm_learner::LinearGaussianNodeState :init
                           :node (key->string (car c))
                           :state (cdr c)))
             alst))
    (t (error "invalid graph-type: ~A" graph-type))))


(defun alist->graph-states (alst &key (graph-type :discrete))
  (let ((cls
          (case graph-type
            (:discrete pgm_learner::DiscreteGraphState)
            (:lg pgm_learner::LinearGaussianGraphState)
            (t (error "invalid graph-type: ~A" graph-type)))))
    (mapcar #'(lambda (trial)
                (instance cls :init
                          :node_states (alist->node-states
                                        trial :graph-type graph-type)))
            alst)))


(defclass bayesian-graph
  :super propertied-object
  :slots (structure nodenames nodedata group-name query-srv-name param-srv-name struct-srv-name))
(defmethod bayesian-graph
  (:init (graph-list grp-name &key (query-p t) (param-p t) (struct-p t))
   (setq group-name grp-name)
   (setq query-srv-name (format nil "~A/query" group-name))
   (setq param-srv-name (format nil "~A/parameter_estimation" group-name))
   (setq struct-srv-name (format nil "~A/structure_estimation" group-name))
   (when query-p
     (ros::wait-for-service query-srv-name 30))
   (when param-p
     (ros::wait-for-service param-srv-name 30))
   (when struct-p
     (ros::wait-for-service struct-srv-name 30))
   (send self :structure graph-list))
  (:structure (&optional graph-list)
   (when graph-list
       (let ((edges (mapcar #'(lambda (gs)
                                (warn "~A -> ~A~%" (car gs) (caddr gs))
                                (instance pgm_learner::GraphEdge :init
                                          :node_from (string-downcase (string (car gs)))
                                          :node_to   (string-downcase (string (caddr gs)))))
                            graph-list)))
         (setq nodes (remove-duplicates
                      (flatten (mapcar #'(lambda (gs)
                                           (list (string-downcase (string (car gs)))
                                                 (string-downcase (string (caddr gs)))))
                                       graph-list))
                      :test #'string=))
         (ros::ros-info "registered ~A nodes" (length nodes))
         (setq structure (instance pgm_learner::GraphStructure :init
                                   :nodes nodes :edges edges))))
       structure)
  (:nodedata (&optional nd)
   (if nd (setq nodedata nd) nodedata))
  (:find-node (name)
   (unless nodedata
     (error "no nodedata")
     (return-from :find-node nil))
   (find-if #'(lambda (x)
                (string= (send x :name) name))
            nodedata)))

(defclass discrete-bayesian-graph
  :super bayesian-graph)
(defmethod discrete-bayesian-graph
  (:init (&optional graph-list (grp-name "pgm_learner"))
   (send-super :init graph-list (format nil "~A/discrete" grp-name)))
  (:parameter-estimation (states)
   (let ((req (instance pgm_learner::DiscreteParameterEstimationRequest :init)))
     (send req :graph structure)
     (send req :states states)
     (setq nodedata (send (ros::service-call param-srv-name req) :nodes))
     nodedata))
  (:query (query &key evidence)
   (let ((req (instance pgm_learner::DiscreteQueryRequest :init)))
     (mapcar #'print-ros-msg (alist->node-states evidence))
     (send req :nodes nodedata)
     (send req :evidence (alist->node-states evidence))
     (send req :query (mapcar #'(lambda (x) (string-downcase (string x))) query))
     (send (ros::service-call query-srv-name req) :nodes)))
  (:structure-estimation (states)
   (let ((req (instance pgm_learner::DiscreteStructureEstimationRequest :init)))
     (send req :states states)
     (setq structure (send (ros::service-call struct-srv-name req) :graph)))))

(defclass lg-bayesian-graph
  :super bayesian-graph)
(defmethod lg-bayesian-graph
  (:init (&optional graph-list (grp-name "pgm_learner"))
   (send-super :init graph-list (format nil "~A/linear_gaussian" grp-name) :query-p nil))
  (:parameter-estimation (states)
   (let ((req (instance pgm_learner::LinearGaussianParameterEstimationRequest :init)))
     (send req :graph structure)
     (send req :states states)
     (setq nodedata (send (ros::service-call param-srv-name req) :nodes))
     nodedata))
  (:structure-estimation (states &key pvalparam bins indegree)
   (let ((req (instance pgm_learner::LinearGaussianStructureEstimationRequest :init)))
     (send req :states states)
     (send req :pvalparam pvalparam)
     (send req :bins bins)
     (send req :indegree indegree)
     (setq structure (send (ros::service-call struct-srv-name req) :graph))))
  (:write-to-dot (&optional (output "graph.dot"))
   (flet ((vabs (v)
            (let ((ret (copy-seq v)))
              (dotimes (i (length ret))
                (setf (aref ret i) (abs (aref ret i))))
              ret)))
     (with-open-file (f output :direction :output)
       (format f "digraph sample {~%")
       (format f "  graph [nodesep = 0.4, ranksep = 1.0];~%")
       (format f "  node [shape = record];~%")
       (dolist (n nodedata)
         (let ((beta-offset (send n :mean)) pname)
           (dotimes (pidx (length (send n :parents)))
             (setq pname (elt (send n :parents) pidx))
             (decf beta-offset (* (elt (send n :mean_scalar) pidx)
                                  (send (send self :find-node pname) :mean))))
           (format f "  ~A [label = \"{~A|mu= ~A\\lvar=~A\\l|beta= ~A\\l}\"~A];~%"
                   (substitute #\_ #\- (send n :name))
                   (send n :name)
                   (send n :mean)
                   (send n :variance)
                   (concatenate float-vector
                                (float-vector beta-offset)
                                (send n :mean_scalar))
                   (if (null (send n :children))
                       ", peripheries = 2" ""))))
       (dolist (n nodedata)
         (let ((cnt 1)
               (max-scal (reduce #'max (vabs
                                        (concatenate float-vector
                                                     (float-vector 0 0)
                                                     (send n :mean_scalar))))))
           (dolist (pname (send n :parents))
             (format f "  ~A -> ~A [headlabel = \" ~A\", labeldistance = 2, penwidth = ~A];~%"
                     (substitute #\_ #\- pname)
                     (substitute #\_ #\- (send n :name))
                     cnt
                     (abs (* 5.0 (/ (elt (send n :mean_scalar)
                                         (position pname (send n :parents)))
                                    max-scal)))
                     )
             (inc cnt))))
       (format f "}~%"))
     t))
  (:write-to-pdf (&optional (output "graph"))
   (let ((dot (concatenate string output ".dot"))
         (pdf (concatenate string output ".pdf")))
     (send self :write-to-dot dot)
     (piped-fork (format nil "dot -Tpdf ~A -o ~A" dot pdf))
     (when (probe-file pdf)
       (piped-fork (format nil "gnome-open ~A" pdf))
       t))))



(provide :pgm-learner-client) ;; end of pgm-learner-client.l
