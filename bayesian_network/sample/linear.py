#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import numpy as np
import pandas as pd
import random

from bayesian_network.linear import *

def cond_normal(m,s):
    return random.gauss(m,s)
np_cond_normal = np.vectorize(cond_normal)

def random_data(n=100):
    A = np.random.normal(2.0, 1.0, n)
    B = np.random.normal(-3.0, 2.0, n)
    C = np_cond_normal(3.0 * A - 2.0 * B, 3.0)
    arr = np.dstack((A,B,C))[0]
    return pd.DataFrame(arr, columns=["A","B","C"])

print "=== generating random data ==="
data = random_data(4000)
print "result:"
print data

print
print "=== estimate graph from data ==="
net = LinearGaussianBayesianNetwork(data=data, debug=True)
print "graph model:", net.model_string()
node_c = [n for n in net.nodes if n.name == "C"][0]
print "node C has mu:", node_c.mu, ", sigma:", node_c.sigma, ", coefficients:", node_c.coefficients

print
print "=== Sampling instance data from given network model ==="
gen_data = net.sample(10)
print "result:"
print gen_data
print gen_data.columns
print gen_data.drop("C", axis=1)

print
print "=== Inferring value given other node values ==="
predicted = net.predict("C", gen_data)
print predicted
