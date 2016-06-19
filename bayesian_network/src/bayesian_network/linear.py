#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


from . import helper
from .exception import *

class LinearGaussianNode(object):
    def __init__(self, name, mu, sigma, parents, children, coefficients):
        self.name = name
        self.mu = mu
        self.sigma = sigma
        self.parents = parents
        self.coefficients = coefficients
    def __repr__(self):
        
