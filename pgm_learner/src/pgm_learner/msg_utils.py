#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import rospy
from pgm_learner.msg import DiscreteNode, LinearGaussianNode, ConditionalProbability, GraphStructure, GraphEdge

from libpgm.graphskeleton import GraphSkeleton
from libpgm.nodedata import NodeData

def graph_skeleton_from_node_data(nd):
    skel = GraphSkeleton()
    skel.V = []
    skel.E = []
    for name, v in nd.Vdata.items():
        skel.V += [name]
        skel.E += [[name, c] for c in v["children"]]
    return skel

def graph_skeleton_from_ros(graph_structure):
    skel = GraphSkeleton()
    skel.V = graph_structure.nodes
    skel.E = [[e.node_from, e.node_to] for e in graph_structure.edges]
    return skel

def graph_skeleton_to_ros(skel):
    graph = GraphStructure()
    if skel.V and len(skel.V) > 0:
        graph.nodes = map(str, skel.V)
    if skel.E and len(skel.E) > 0:
        graph.edges = [GraphEdge(str(e[0]),str(e[1])) for e in skel.E]
    return graph

def graph_state_dict_from_ros(graph_state):
    data = {}
    for s in graph_state.node_states:
        data[s.node] = s.state
    return data

def graph_states_dict_from_ros(graph_states):
    return [graph_state_dict_from_ros(gs) for gs in graph_states]

def discrete_node_from_dict(name, d):
    n = DiscreteNode()
    n.name = str(name)
    n.outcomes = map(str, d["vals"])
    if d["parents"]:
        n.parents = map(str, d["parents"])
    if d["children"]:
        n.children = map(str, d["children"])
    cprob = d["cprob"]
    if isinstance(cprob, dict):
        n.CPT = [ConditionalProbability(values=eval(k), probabilities=v) for k,v in (d["cprob"]).items()]
    else:
        n.CPT = [ConditionalProbability(values=map(str, d["vals"]), probabilities=cprob)]
    return n

def discrete_nodes_to_ros(d):
    return [discrete_node_from_dict(k, v) for k,v in d.items()]

def dict_from_ros_discrete_node(msg):
    d = {}
    d["vals"] = msg.outcomes
    d["numoutcomes"] = len(msg.outcomes)
    d["parents"] = msg.parents
    d["children"] = msg.children
    if len(msg.CPT) == 1:
        d["cprob"] = msg.CPT[0].probabilities
    else:
        d["cprob"] = {str(p.values): p.probabilities for p in msg.CPT}
    return d


def discrete_nodedata_from_ros(nodes):
    nd = NodeData()
    nd.Vdata = {n.name: dict_from_ros_discrete_node(n) for n in nodes}
    return nd

def linear_gaussian_node_from_dict(name, d):
    n = LinearGaussianNode()
    n.name = str(name)
    if d["parents"]:
        n.parents = map(str, d["parents"])
    if d["children"]:
        n.children = map(str, d["children"])
    n.mean = d["mean_base"]
    n.variance = d["variance"]
    n.mean_scalar = d["mean_scal"]
    return n

def linear_gaussian_nodes_to_ros(d):
    return [linear_gaussian_node_from_dict(k,v) for k,v in d.items()]
