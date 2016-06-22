#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import rospy
from bayesian_network.srv import *
from bayesian_network.msg import *


class BayesianNetworkServer(object):
    def __init__(self):
        pass

if __name__ == '__main__':
    rospy.init_node("bayesian_network_server")
    s = BayesianNetworkServer()
    rospy.spin()
