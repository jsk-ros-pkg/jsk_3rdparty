#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import itertools as it

from libpgm.discretebayesiannetwork import DiscreteBayesianNetwork
from libpgm.graphskeleton import GraphSkeleton
from libpgm.nodedata import NodeData
from libpgm.tablecpdfactorization import TableCPDFactorization

import rospy
from pgm_learner.msg import DiscreteNode, DiscreteNodeState, ConditionalProbability
from pgm_learner.srv import DiscreteQuery, DiscreteQueryRequest

def q_without_ros():
    skel = GraphSkeleton()
    skel.V = ["prize_door", "guest_door", "monty_door"]
    skel.E = [["prize_door", "monty_door"],
              ["guest_door", "monty_door"]]
    skel.toporder()
    nd = NodeData()
    nd.Vdata = {
        "prize_door": {
            "numoutcomes": 3,
            "parents": None,
            "children": ["monty_door"],
            "vals": ["A", "B", "C"],
            "cprob": [1.0/3, 1.0/3, 1.0/3],
        },
        "guest_door": {
            "numoutcomes": 3,
            "parents": None,
            "children": ["monty_door"],
            "vals": ["A", "B", "C"],
            "cprob": [1.0/3, 1.0/3, 1.0/3],
        },
        "monty_door": {
            "numoutcomes": 3,
            "parents": ["prize_door", "guest_door"],
            "children": None,
            "vals": ["A", "B", "C"],
            "cprob": {
                "['A', 'A']": [0., 0.5, 0.5],
                "['B', 'B']": [0.5, 0., 0.5],
                "['C', 'C']": [0.5, 0.5, 0.],
                "['A', 'B']": [0., 0., 1.],
                "['A', 'C']": [0., 1., 0.],
                "['B', 'A']": [0., 0., 1.],
                "['B', 'C']": [1., 0., 0.],
                "['C', 'A']": [0., 1., 0.],
                "['C', 'B']": [1., 0., 0.],
            },
        },
    }
    bn = DiscreteBayesianNetwork(skel, nd)
    fn = TableCPDFactorization(bn)

    query = {
        "prize_door": ["A","B","C"],
    }
    evidence = {
        "guest_door": "A",
        "monty_door": "B",
    }

    res = fn.condprobve(query, evidence)
    print(res.vals)
    print(res.scope)
    print(res.card)
    print(res.stride)

def monty_door_prob(prize, guest):
    if prize == guest:
        ret = [0.5,0.5]
        ret.insert(ord(prize)-ord("A"), 0.)
    else:
        s = set(["A","B","C"])
        d = s.difference([prize, guest])
        pos = ord(d.pop()) - ord("A")
        ret = [0.,0.]
        ret.insert(pos, 1.)
    return ret

def q():
    rospy.init_node("discrete_bayesian_query_sample")
    query_func = rospy.ServiceProxy("pgm_learner/discrete/query", DiscreteQuery)


    prize_door = DiscreteNode()
    prize_door.name = "prize_door"
    prize_door.children = ["monty_door"]
    prize_door.outcomes = ["A", "B", "C"]
    prize_door.CPT = [ConditionalProbability(values=prize_door.outcomes,
                                             probabilities=[1.0/3,1.0/3,1.0/3])]
    guest_door = DiscreteNode()
    guest_door.name = "guest_door"
    guest_door.children = ["monty_door"]
    guest_door.outcomes = ["A", "B", "C"]
    guest_door.CPT = [ConditionalProbability(values=guest_door.outcomes,
                                             probabilities=[1.0/3,1.0/3,1.0/3])]
    monty_door = DiscreteNode()
    monty_door.name = "monty_door"
    monty_door.parents = ["prize_door", "guest_door"]
    monty_door.outcomes = ["A", "B", "C"]
    for prize, guest in it.product(["A","B","C"], repeat=2):
        print(str([prize, guest]))
        print(monty_door_prob(prize, guest))
        monty_door.CPT += [ConditionalProbability(values=[prize, guest],
                                                  probabilities=monty_door_prob(prize=prize, guest=guest))]

    req = DiscreteQueryRequest()
    req.nodes = [prize_door, guest_door, monty_door]
    req.evidence = [DiscreteNodeState(node="guest_door", state="A"),
                    DiscreteNodeState(node="monty_door", state="B"),]
    req.query = ["prize_door"]
    print(query_func(req))

if __name__ == '__main__':
    q()
