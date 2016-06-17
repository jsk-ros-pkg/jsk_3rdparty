#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from bayesian_network.discrete import *
import numpy as np

values = ["A","B","C"]
nodes = [
    DiscreteNode("prize_door", values,
                 None, ["monty_door"],
                 np.array([1./3]*3)),
    DiscreteNode("guest_door", values,
                 None, ["monty_door"],
                 np.array([1./3]*3)),
    DiscreteNode("monty_door", values,
                 ["prize_door", "guest_door"], None,
                 np.array([0.,.5,.5,
                           0.,0.,1.,
                           0.,1.,0.,
                           0.,0.,1.,
                           .5,0.,.5,
                           1.,0.,0.,
                           0.,1.,0.,
                           1.,0.,0.,
                           .5,.5,0.])),
]

net = DiscreteBayesianNetwork(nodes)
net.update_cpts()
res = net.query([["guest_door", "A"],
                 ["prize_door", "A"]],
                "monty_door")
print res
