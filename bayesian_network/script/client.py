#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
from bayesian_network.msg import DiscreteNode, Evidence, NodeValue
from bayesian_network.srv import *
from bayesian_network.srv import StructureEstimationRequest as S
import bayesian_network.discrete as D
import bayesian_network.linear as L
import numpy as np
import pandas as pd

def monty_hall():
    query = rospy.ServiceProxy("/query", Query)

    values = ["A","B","C"]
    req = QueryRequest()
    req.nodes = [
        D.DiscreteNode("prize_door", values,
                       None, ["monty_door"],
                       np.array([1./3] * 3)).to_ros(),
        D.DiscreteNode("guest_door", values,
                       None, ["monty_door"],
                       np.array([1./3] * 3)).to_ros(),
        D.DiscreteNode("monty_door", values,
                       ["prize_door", "guest_door"], None,
                       np.array([0,.5,.5,
                                 0,0,1,
                                 0,1,0,
                                 .5,0,.5,
                                 1,0,0,
                                 0,1,0,
                                 1,0,0,
                                 .5,.5,0])).to_ros()
        ]
    req.evidences = [
        Evidence(values=[NodeValue(name="guest_door", discrete_value="A"),
                         NodeValue(name="prize_door", discrete_value="A")])
    ]
    req.query_node = "monty_door"
    res = query(req)
    print res.cdist, res.cprob


if __name__ == '__main__':
    rospy.init_node("bayesian_network_client_sample")
    monty_hall()
