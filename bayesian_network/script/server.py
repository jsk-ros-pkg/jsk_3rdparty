#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import rospy
from bayesian_network.srv import *
from bayesian_network.srv import StructureEstimationRequest as S
from bayesian_network.msg import *
import bayesian_network.discrete as D
import bayesian_network.linear as L
import pandas as pd

class BayesianNetworkServer(object):
    def __init__(self):
        self.predict_srv = rospy.Service("predict", Predict, self.predict_cb)
        self.query_srv = rospy.Service("query", Query, self.query_cb)
        self.sample_srv = rospy.Service("sample", Sample, self.sample_cb)
        self.structure_estimation_srv = rospy.Service("structure_estimation",
                                                      StructureEstimation,
                                                      self.structure_estimation_cb)
    def predict_cb(self, req):
        nodes = [L.from_ros(n) for n in req.nodes]
        net = L.LinearGaussianBayesianNetwork(nodes)
        node_name = req.node_name
        data = pd.DataFrame()
        for e in req.evidences:
            values = pd.DataFrame({v.name: v.continuous_value for v in e.values})
            data.append(values)
        res = net.predict(node_name, data)
        print res
    def query_cb(self, req):
        nodes = [D.from_ros(n) for n in req.nodes]
        net = D.DiscreteBayesianNetwork(nodes)
        data = pd.DataFrame()
        for e in req.evidences:
            values = pd.DataFrame({v.name: v.discrete_value for v in e.values})
            data.append(values)
        if req.query_node is not None or len(req.query_node) > 0:
            res = net.query(data, req.query_node)
        else:
            q = pd.DataFrame({v.name: v.discrete_value for v in req.query_state.values})
            res = net.query(data, q)
        print res
    def sample_cb(self, req):
        if len(req.discrete_nodes) > 0:
            nodes = [D.from_ros(n) for n in req.discrete_nodes]
            net = D.DiscreteBayesianNetwork(nodes)
        else:
            nodes = [L.from_ros(n) for n in req.linear_gaussian_nodes]
            net = L.LinearGaussianBayesianNetwork(nodes)
        res = net.sample(req.sample_num)
        print res
    def structure_estimation_cb(self, req):
        data = pd.DataFrame()
        method = self.method_string(req.method)
        if req.discrete:
            for e in req.evidences:
                values = pd.DataFrame({v.name: v.discrete_value for v in e.values})
                data.append(values)
            net = D.DiscreteBayesianNetwork(data=data, method=method)
        else:
            for e in req.evidences:
                values = pd.DataFrame({v.name: v.continuous_value for v in e.values})
                data.append(values)
            net = L.LinearGaussianBayesianNetwork(data=data, method=method)
        print net.model_string()
    def method_string(self, m):
        if m == S.gs:
            return "gs"
        elif m == S.iamb:
            return "iamb"
        elif m == S.fast_iamb:
            return "fast.iamb"
        elif m == S.inter_iamb:
            return "inter.iamb"
        elif m == S.hc:
            return "hc"
        elif m == S.tabu:
            return "tabu"
        elif m == S.mmhc:
            return "mmhc"
        elif m == S.rsmax2:
            return "rsmax2"
        elif m == "mmpc":
            return "mmpc"
        elif m == S.si_hiton_pc:
            return "si.hiton.pc"
        elif m == S.chow_liu:
            return "chow.liu"
        elif m == S.aracne:
            return "aracne"
        else:
            return "hc"

if __name__ == '__main__':
    rospy.init_node("bayesian_network")
    s = BayesianNetworkServer()
    rospy.spin()
