#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from __future__ import print_function
import os
import sys
import unittest
import rospy
import rospkg
from pgm_learner.srv import (DiscreteParameterEstimation,
                             DiscreteParameterEstimationRequest,
                             DiscreteQuery,
                             DiscreteQueryRequest,
                             DiscreteStructureEstimation,
                             DiscreteStructureEstimationRequest,
                             )
from pgm_learner.msg import GraphEdge, DiscreteGraphState, DiscreteNodeState
import pgm_learner.msg_utils as U

from libpgm.graphskeleton import GraphSkeleton
from libpgm.nodedata import NodeData
from libpgm.discretebayesiannetwork import DiscreteBayesianNetwork


class TestDiscreteBNLearnerNode(unittest.TestCase):
    def __init__(self, arg):
        super(self.__class__, self).__init__(arg)
        self.pkg = rospkg.RosPack()
        self.data_path = os.path.join(self.pkg.get_path("pgm_learner"), "test", "graph-test.txt")
        self.teacher_data_path = self.data_path
        self.param_estimate = rospy.ServiceProxy("pgm_learner/discrete/parameter_estimation", DiscreteParameterEstimation)
        self.query = rospy.ServiceProxy("pgm_learner/discrete/query", DiscreteQuery)
        self.struct_estimate = rospy.ServiceProxy("pgm_learner/discrete/structure_estimation", DiscreteStructureEstimation)
        self.param_estimate.wait_for_service(timeout=30)
        self.query.wait_for_service(timeout=30)
        self.struct_estimate.wait_for_service(timeout=30)
    def test_param_estimation(self):
        req = DiscreteParameterEstimationRequest()

        # load graph structure
        skel = GraphSkeleton()
        skel.load(self.data_path)
        req.graph.nodes = skel.V
        req.graph.edges = [GraphEdge(k, v) for k,v in skel.E]
        skel.toporder()

        # generate trial data
        teacher_nd = NodeData()
        teacher_nd.load(self.teacher_data_path)
        bn = DiscreteBayesianNetwork(skel, teacher_nd)
        data = bn.randomsample(200)
        for v in data:
            gs = DiscreteGraphState()
            for k_s, v_s in v.items():
                gs.node_states.append(DiscreteNodeState(node=k_s, state=v_s))
            req.states.append(gs)

        self.assertEqual(len(self.param_estimate(req).nodes), 5)

    def test_query(self):
        teacher_nd = NodeData()
        teacher_nd.load(self.teacher_data_path)
        req = DiscreteQueryRequest()
        req.nodes = U.discrete_nodes_to_ros(teacher_nd.Vdata)
        req.evidence = [DiscreteNodeState("Letter", "weak")]
        req.query = ["Grade"]
        res = self.query(req)
        self.assertEqual(len(res.nodes), 1)
        n = res.nodes[0]
        self.assertEqual(n.name, "Grade")
        self.assertListEqual(['A','B','C'], n.outcomes)

    def test_structure_estimation(self):
        req = DiscreteStructureEstimationRequest()

        skel = GraphSkeleton()
        skel.load(self.data_path)
        skel.toporder()
        teacher_nd = NodeData()
        teacher_nd.load(self.teacher_data_path)
        bn = DiscreteBayesianNetwork(skel, teacher_nd)
        data = bn.randomsample(8000)
        for v in data:
            gs = DiscreteGraphState()
            for k_s, v_s in v.items():
                gs.node_states.append(DiscreteNodeState(node=k_s, state=v_s))
            req.states.append(gs)

        res = self.struct_estimate(req)
        self.assertIsNotNone(res.graph)
        self.assertEqual(len(res.graph.nodes), 5)
        self.assertGreater(len(res.graph.edges), 0)

if __name__ == '__main__':
    test_name = "test_discrete_bayesian_network_learner"
    rospy.init_node(test_name)
    import rostest
    rostest.rosrun("pgm_learner", test_name, TestDiscreteBNLearnerNode)

