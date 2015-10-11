#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import os
import pprint
import rospkg
import rospy
from pgm_learner.srv import LinearGaussianParameterEstimation, LinearGaussianParameterEstimationRequest
from pgm_learner.msg import GraphEdge, LinearGaussianGraphState, LinearGaussianNodeState

from libpgm.graphskeleton import GraphSkeleton
from libpgm.nodedata import NodeData
from libpgm.lgbayesiannetwork import LGBayesianNetwork

PKG_PATH = rospkg.RosPack().get_path("pgm_learner")
PP = pprint.PrettyPrinter(indent=2)

if __name__ == '__main__':
    rospy.init_node("pgm_learner_sample_linear_gaussian")

    param_estimate = rospy.ServiceProxy("pgm_learner/linear_gaussian/parameter_estimation", LinearGaussianParameterEstimation)

    req = LinearGaussianParameterEstimationRequest()

    dpath = os.path.join(PKG_PATH, "test", "graph-test.txt")
    tpath = os.path.join(PKG_PATH, "test", "graph-lg-test.txt")

    # load graph structure
    skel = GraphSkeleton()
    skel.load(dpath)
    req.graph.nodes = skel.V
    req.graph.edges = [GraphEdge(k, v) for k,v in skel.E]
    skel.toporder()

    # generate trial data
    teacher_nd = NodeData()
    teacher_nd.load(tpath)
    bn = LGBayesianNetwork(skel, teacher_nd)
    data = bn.randomsample(200)

    for v in data:
        gs = LinearGaussianGraphState()
        for k_s, v_s in v.items():
            gs.node_states.append(LinearGaussianNodeState(node=k_s, state=v_s))
        req.states.append(gs)

    PP.pprint(param_estimate(req).nodes)
