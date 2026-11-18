#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


from . import helper
from .exception import *
import numpy as np
import pandas as pd
import bayesian_network.msg as MSG


class LinearGaussianNode(helper.DAGNode):
    def __init__(self, name, mu, sigma, parents, children, coefficients):
        super(LinearGaussianNode, self).__init__(name, parents, children)
        self.mu = mu
        self.sigma = sigma
        self.coefficients = self._check_val(coefficients)
    def to_msg(self):
        msg = MSG.LinearGaussianNode()
        msg.name = self.name
        msg.parents = self.parents
        msg.children = self.children
        msg.mu = self.mu
        msg.sigma = self.sigma
        msg.coefficients = self.coefficients
        return msg

def from_ros(msg):
    return LinearGaussianNode(
        msg.name,
        msg.mu,
        msg.sigma,
        np.array(msg.parents),
        np.array(msg.children),
        np.array(msg.coefficients))

class LinearGaussianBayesianNetwork(helper.BNLearnObject):
    def __init__(self, nodes=None, data=None, debug=False, **kwargs):
        super(LinearGaussianBayesianNetwork, self).__init__(debug)
        if nodes is not None:
            self.add_nodes(nodes)
        elif data is not None:
            try:
                method=kwargs["method"]
            except:
                method="hc"
            self.estimate_network(data, method, **kwargs)
    def _parse_graph(self):
        self.ensure("fit")
        nodenames = self.r["nodes(fit)"]
        self.nodes = []
        for name in nodenames:
            parents = self.r["fit$%s$parents" % name]
            children = self.r["fit$%s$children" % name]
            sigma = self.r["fit$%s$sd" % name]
            coeffs = self.r["fit$%s$coefficients" % name]
            if type(coeffs) is float:
                # no parent
                mu = coeffs
                coeffs = []
            else:
                mu = coeffs[0]
                coeffs = coeffs[1:]
            self.nodes += [LinearGaussianNode(name, mu, sigma, parents, children, coeffs)]
    def sample(self, n):
        self.ensure("fit")
        self.rcmd("sample.result <- rbn(fit, %d)" % n)
        return self.r["sample.result"].rename(columns=lambda x: x.strip())
    def predict(self, node_name, data):
        self.ensure("fit")
        self.r["predict.data"] = data
        self.rcmd('predict.result <- predict(fit, "%s", predict.data)' % node_name)
        return self.r["predict.result"]
