#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


from bayesian_network.discrete import *
import numpy as np
import pandas as pd
from collections import OrderedDict
from operator import itemgetter
import math
try:
    import colorama
except:
    print """Please install colorama by
pip install colorama"""
    sys.exit(1)
from colorama import Fore, Style

colorama.init()
def ask(*msg):
    return raw_input(Style.BRIGHT + Fore.CYAN + ' '.join([str(m) for m in msg]) + "?: " + Fore.RESET + Style.RESET_ALL)
def ok(*msg):
    print Style.BRIGHT + Fore.GREEN + ' '.join([str(m) for m in msg]) + Fore.RESET + Style.RESET_ALL
def info(*msg):
    print ' '.join([str(m) for m in msg])
def warn(*msg):
    print Style.BRIGHT + Fore.YELLOW + ' '.join([str(m) for m in msg]) + Fore.RESET + Style.RESET_ALL
def err(*msg):
    print Style.BRIGHT + Fore.RED + ' '.join([str(m) for m in msg]) + Fore.RESET + Style.RESET_ALL

## 1. define nodes
objects = sorted(["georgia", "cafe_au_lait", "bigmac", "iemon", "wonda", "mug"])
values = OrderedDict([
    ['color', sorted(["blue", "red", "green", "white", "orange","brown"])],
    ['size', sorted(["big", "mid", "tall"])],
    ['shape', sorted(["box", "cylinder", "polyhedron"])],
    ['type', sorted(["food", "drink", "can", "cup"])],
])

def prop_node(name, val, par=["object"]):
    return DiscreteNode(name, val, par, None, None)

nodes = [
    DiscreteNode("object", objects,
                 None, values.keys(), None)
]
nodes += [ prop_node(k, v) for k,v in values.items() ]

## 2. create bayesian network instance
net = DiscreteBayesianNetwork(nodes, debug=False)

## 3. learn network by inputing evidences
# ["object", "color", "size", "shape", "type"]
data = pd.DataFrame(np.array([
    ["georgia", "blue", "mid", "cylinder", "can"],
    ["cafe_au_lait", "white", "mid", "cylinder", "can"],
    ["bigmac", "orange", "big", "polyhedron", "food"],
    ["iemon", "green", "tall", "box", "pack"],
    ["wonda", "red", "mid", "cylinder", "can"],
    ["mug", "brown", "big", "cylinder", "cup"],
]), columns=["object"] + values.keys())

net.fit(data)

## 4. infer by giving evidences and queries
def query_object(q):
    print
    info("you are asking:", q)
    res = net.query(q,"object")
    info("infered probability is", res)
    ok("assumed you asked about", objects[np.argmax(res)])

def conversation():
    print
    ok("hi!")

    qkey = None
    qval = None
    queries = []
    while True:
        if qval is None:
            qval = ask("What do you want")
        for k,v in values.items():
            if qval in v:
                qkey = k
                break
        if qkey is None:
            err("sorry, I don't know...")
            qval = None
            continue
        info("you are asking about", qkey)
        info("Hmm...")
        queries.append([qkey, qval])
        res = net.query(queries, "object")
        rank = sorted(res, reverse=True)
        if abs(rank[0] - rank[1]) > 0.1:
            answer = objects[np.argmax(res)]
            ok("So, you want", answer, "right?")
            return answer
        elif math.isnan(rank[0]):
            err("Maybe there is nothing you want...")
            ok("Let's try again from start!")
            qkey = None
            qval = None
            queries = []
        else:
            warn("Well, your question is still ambiguous.")
            keys = [e[0] for e in queries]
            var = {k: np.var(net.query(queries, k)) for k,v in values.items() if k not in keys }
            qkey = min(var.items(), key=itemgetter(1))[0]
            qval = ask("Tell me how about", qkey)

net.plot()

query_object([["color", "red"]])
query_object([["type", "can"]])

conversation()
