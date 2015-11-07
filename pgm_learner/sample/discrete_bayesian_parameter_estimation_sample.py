#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import os
import pprint
import rospkg
import rospy
from pgm_learner.srv import DiscreteParameterEstimation, DiscreteParameterEstimationRequest
from pgm_learner.msg import GraphEdge, DiscreteGraphState, DiscreteNodeState

from libpgm.graphskeleton import GraphSkeleton
from libpgm.nodedata import NodeData
from libpgm.discretebayesiannetwork import DiscreteBayesianNetwork

PKG_PATH = rospkg.RosPack().get_path("pgm_learner")
PP = pprint.PrettyPrinter(indent=2)

if __name__ == '__main__':
    rospy.init_node("pgm_learner_sample_discrete")

    param_estimate = rospy.ServiceProxy("pgm_learner/discrete/parameter_estimation", DiscreteParameterEstimation)

    req = DiscreteParameterEstimationRequest()

    dpath = os.path.join(PKG_PATH, "test", "graph-test.txt")
    tpath = dpath

    # load graph structure
    skel = GraphSkeleton()
    skel.load(dpath)
    req.graph.nodes = skel.V
    req.graph.edges = [GraphEdge(k, v) for k,v in skel.E]
    skel.toporder()

    # generate trial data
    teacher_nd = NodeData()
    teacher_nd.load(dpath)
    bn = DiscreteBayesianNetwork(skel, teacher_nd)
    data = bn.randomsample(200)
    for v in data:
        gs = DiscreteGraphState()
        for k_s, v_s in v.items():
            gs.node_states.append(DiscreteNodeState(node=k_s, state=v_s))
        req.states.append(gs)

    PP.pprint(param_estimate(req).nodes)
