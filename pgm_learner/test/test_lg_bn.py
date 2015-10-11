#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import os
import unittest
import rospy
import rospkg
from pgm_learner.srv import (LinearGaussianParameterEstimation,
                             LinearGaussianParameterEstimationRequest,
                             LinearGaussianStructureEstimation,
                             LinearGaussianStructureEstimationRequest,
                             )
from pgm_learner.msg import GraphEdge, LinearGaussianGraphState, LinearGaussianNodeState

from libpgm.graphskeleton import GraphSkeleton
from libpgm.nodedata import NodeData
from libpgm.lgbayesiannetwork import LGBayesianNetwork


class TestLGBNLearnerNode(unittest.TestCase):
    def __init__(self, arg):
        super(self.__class__, self).__init__(arg)
        self.pkg = rospkg.RosPack()
        self.data_path = os.path.join(self.pkg.get_path("pgm_learner"), "test", "graph-test.txt")
        self.teacher_data_path = os.path.join(self.pkg.get_path("pgm_learner"), "test", "graph-lg-test.txt")
        self.param_estimate = rospy.ServiceProxy("pgm_learner/linear_gaussian/parameter_estimation", LinearGaussianParameterEstimation)
        self.struct_estimate = rospy.ServiceProxy("pgm_learner/linear_gaussian/structure_estimation", LinearGaussianStructureEstimation)
        self.param_estimate.wait_for_service(timeout=30)
        self.struct_estimate.wait_for_service(timeout=30)

    def test_param_estimation(self):
        req = LinearGaussianParameterEstimationRequest()

        # load graph structure
        skel = GraphSkeleton()
        skel.load(self.data_path)
        req.graph.nodes = skel.V
        req.graph.edges = [GraphEdge(k, v) for k,v in skel.E]
        skel.toporder()

        # generate trial data
        teacher_nd = NodeData()
        teacher_nd.load(self.teacher_data_path)
        bn = LGBayesianNetwork(skel, teacher_nd)
        data = bn.randomsample(200)
        for v in data:
            gs = LinearGaussianGraphState()
            for k_s, v_s in v.items():
                gs.node_states.append(LinearGaussianNodeState(node=k_s, state=v_s))
            req.states.append(gs)

        self.assertEqual(len(self.param_estimate(req).nodes), 5)

    def test_structure_estimation(self):
        req = LinearGaussianStructureEstimationRequest()

        # generate trial data
        skel = GraphSkeleton()
        skel.load(self.data_path)
        skel.toporder()
        teacher_nd = NodeData()
        teacher_nd.load(self.teacher_data_path)
        bn = LGBayesianNetwork(skel, teacher_nd)
        data = bn.randomsample(8000)
        for v in data:
            gs = LinearGaussianGraphState()
            for k_s, v_s in v.items():
                gs.node_states.append(LinearGaussianNodeState(node=k_s, state=v_s))
            req.states.append(gs)

        res = self.struct_estimate(req)
        self.assertIsNotNone(res.graph)
        self.assertEqual(len(res.graph.nodes), 5)
        self.assertEqual(len(res.graph.edges), 4)


if __name__ == '__main__':
    test_name = "test_linear_gaussian_bayesian_network_learner"
    rospy.init_node(test_name)
    import rostest
    rostest.rosrun("pgm_learner", test_name, TestLGBNLearnerNode)

