#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from pyper import R

class RObject(object):
    def __init__(self, debug=False):
        self.r = R(use_pandas=True, use_numpy=True)
        self.debug = debug
    def rcmd(self, cmd):
        if self.debug:
            print(cmd)
        res = self.r(cmd)
        if 'Error' in res:
            raise Exception(res)
        return res
    def plot(self):
        self.rcmd('plot(graph)')
