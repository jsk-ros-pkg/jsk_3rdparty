#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import rospy
from pgm_learner.msg import DiscreteNode, ConditionalProbability
from pgm_learner.srv import (DiscreteParameterEstimation,
                             DiscreteParameterEstimationResponse,
                             DiscreteQuery,
                             DiscreteQueryResponse,
                             DiscreteStructureEstimation,
                             DiscreteStructureEstimationResponse,
                             LinearGaussianParameterEstimation,
                             LinearGaussianParameterEstimationResponse,
                             LinearGaussianStructureEstimation,
                             LinearGaussianStructureEstimationResponse,
                             )
import pgm_learner.msg_utils as U

from libpgm.nodedata import NodeData
from libpgm.graphskeleton import GraphSkeleton
from libpgm.discretebayesiannetwork import DiscreteBayesianNetwork
from libpgm.pgmlearner import PGMLearner
from libpgm.tablecpdfactorization import TableCPDFactorization


class PGMLearnerServer(object):
    def __init__(self):
        self.learner = PGMLearner()
        rospy.Service("~discrete/parameter_estimation", DiscreteParameterEstimation, self.discrete_parameter_estimation_cb)
        rospy.Service("~discrete/query", DiscreteQuery, self.discrete_query_cb)
        rospy.Service("~discrete/structure_estimation", DiscreteStructureEstimation, self.discrete_structure_estimation_cb)
        rospy.Service("~linear_gaussian/parameter_estimation", LinearGaussianParameterEstimation, self.lg_parameter_estimation_cb)
        rospy.Service("~linear_gaussian/structure_estimation", LinearGaussianStructureEstimation, self.lg_structure_estimation_cb)

    def discrete_parameter_estimation_cb(self, req):
        skel = U.graph_skeleton_from_ros(req.graph)
        skel.toporder()
        data = U.graph_states_dict_from_ros(req.states)
        res = self.learner.discrete_mle_estimateparams(skel, data)
        return DiscreteParameterEstimationResponse(U.discrete_nodes_to_ros(res.Vdata))

    def discrete_query_cb(self, req):
        nd = U.discrete_nodedata_from_ros(req.nodes)
        skel = U.graph_skeleton_from_node_data(nd)
        skel.toporder()
        bn = DiscreteBayesianNetwork(skel, nd)
        fn = TableCPDFactorization(bn)
        q = {n: nd.Vdata[n]["vals"] for n in req.query}
        ev = {ns.node: ns.state for ns in req.evidence}

        rospy.loginfo("resolving query %s with evidence %s" % (q, ev))
        ans = fn.condprobve(query=q, evidence=ev)
        rospy.loginfo("%s -> %s" % (ans.scope, ans.vals))
        res = DiscreteQueryResponse()
        node = DiscreteNode()
        node.name = ans.scope[0]
        node.outcomes=q[node.name]
        node.CPT.append(ConditionalProbability(node.outcomes, ans.vals))
        res.nodes.append(node)
        return res

    def discrete_structure_estimation_cb(self, req):
        states = [{ns.node: ns.state for ns in s.node_states} for s in req.states]
        pvalparam = 0.05 # default value
        indegree = 1 # default value
        if req.pvalparam != 0.0:
            pvalparam = req.pvalparam
        if req.indegree != 0:
            indegree = req.indegree
        res = self.learner.discrete_constraint_estimatestruct(states,
                                                              pvalparam=pvalparam,
                                                              indegree=indegree)
        return DiscreteStructureEstimationResponse(U.graph_skeleton_to_ros(res))

    def lg_parameter_estimation_cb(self, req):
        skel = U.graph_skeleton_from_ros(req.graph)
        skel.toporder()
        data = U.graph_states_dict_from_ros(req.states)
        res = self.learner.lg_mle_estimateparams(skel, data)
        rospy.logdebug("parameter estimation: %s" % res.Vdata)
        return LinearGaussianParameterEstimationResponse(U.linear_gaussian_nodes_to_ros(res.Vdata))

    def lg_structure_estimation_cb(self, req):
        states = [{ns.node: ns.state for ns in s.node_states} for s in req.states]
        rospy.logdebug(states)
        pvalparam = 0.05 # default value
        bins = 10 # default value
        indegree = 1 # default value
        if req.pvalparam != 0.0:
            pvalparam = req.pvalparam
        if req.bins != 0:
            bins = req.bins
        if req.indegree != 0:
            indegree = req.indegree
        rospy.logdebug("bins: %d, pvalparam: %f, indegree: %d" % (bins, pvalparam, indegree))
        res = self.learner.lg_constraint_estimatestruct(states,
                                                        pvalparam=pvalparam,
                                                        bins=bins,
                                                        indegree=indegree)
        rospy.logdebug("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        rospy.logdebug(res.V)
        rospy.logdebug(res.E)
        return LinearGaussianStructureEstimationResponse(U.graph_skeleton_to_ros(res))


if __name__ == '__main__':
    rospy.init_node("pgm_learner")
    n = PGMLearnerServer()
    rospy.spin()
