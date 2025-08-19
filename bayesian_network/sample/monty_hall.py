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

print
print "=== Inferring prob of monty_door given guest_door=A & prize_door=A ==="
net = DiscreteBayesianNetwork(nodes)
res = net.query([["guest_door", "A"],
                 ["prize_door", "A"]],
                "monty_door")
print "result:", res

print
print "=== Inferring prob of monty_door==C given guest_door=A & prize_door=A ==="
res2 = net.query([["guest_door", "A"],
                  ["prize_door", "B"]],
                  [["monty_door", "C"]])
print "result:", res2

print
print "=== Sampling instance data from given network model ==="
data = net.sample(1000)
print "result:", data[:10], "...", len(data), "data"

print
print "=== Estimating network from data of monty hall ==="
net2 = DiscreteBayesianNetwork(data=data)
print "result:", net2.model_string()
net2.plot(to_pdf="monty_hall.pdf")
print "saved figure to monty_hall.pdf"
net2.plot_node("prize_door")
