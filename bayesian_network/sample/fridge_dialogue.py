#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


from bayesian_network.discrete import *
import numpy as np
import pandas as pd
from collections import OrderedDict
from operator import itemgetter

objects = sorted(["georgia", "cafe_au_lait", "bigmac", "iemon", "wonda", "mug"])
values = OrderedDict([
    ['color', sorted(["blue", "red", "green", "white", "orange","brown"])],
    ['size', sorted(["big", "mid", "tall"])],
    ['shape', sorted(["box", "cylinder", "polyhedron"])],
    ['type', sorted(["food", "drink", "cup"])],
])

def prop_node(name, val, par=["object"]):
    return DiscreteNode(name, val, par, None, None)

nodes = [
    DiscreteNode("object", objects,
                 None, values.keys(), None)
]
nodes += [ prop_node(k, v) for k,v in values.items() ]

net = DiscreteBayesianNetwork(nodes, debug=True)

# ["object", "color", "size", "shape", "type"]
data = pd.DataFrame(np.array([
    ["georgia", "blue", "mid", "cylinder", "can"],
    ["cafe_au_lait", "white", "mid", "cylinder", "can"],
    ["bigmac", "orange", "big", "polyhedron", "food"],
    ["iemon", "green", "tall", "box", "pack"],
    ["wonda", "red", "mid", "cylinder", "can"],
    ["mug", "brown", "big", "cylinder", "cup"],
]*10), columns=["object"] + values.keys())

print data

print net.fit(data)

def ask(q):
    print
    print "you are asking:", q
    res = net.query(q,"object")
    print "infered probability is", res
    print "assumed you asked about", objects[np.argmax(res)]

def conversation():
    print
    print "hi!"

    qkey = None
    qval = None
    queries = []
    while True:
        if qval is None:
            qval = raw_input("What do you want?: ")
        for k,v in values.items():
            if qval in v:
                qkey = k
                break
        if qkey is None:
            print "sorry, I don't know..."
            qval = None
            continue
        print "you are asking about", qkey
        print "Hmm..."
        queries.append([qkey, qval])
        res = net.query(queries, "object")
        print "query:", queries, "result:", res
        rank = sorted(res, reverse=True)
        if abs(rank[0] - rank[1]) > 0.1:
            answer = objects[np.argmax(res)]
            print "So, you want", answer, "right?"
            return answer
        else:
            print "Well, your question is still ambiguous."
            keys = [e[0] for e in queries]
            var = {k: np.var(net.query(queries, k)) for k,v in values.items() if k not in keys }
            qkey = min(var.items(), key=itemgetter(1,1))[0]
            qval = raw_input("Tell me how about " + qkey + "?: ")

net.plot()

ask([["color", "red"]])
ask([["type", "can"]])

conversation()
