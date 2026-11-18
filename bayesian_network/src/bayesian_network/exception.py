#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


class ValueNotDefinedException(Exception):
    hints = {
        'graph': 'try add_nodes method',
        'fit': 'try fit method'
    }
    def __init__(self, varname):
        self.varname = varname
    def __str__(self):
        hint = ''
        if self.varname in hints:
            hint = "(" + hints[self.varname] + ")"
        return "Value %s is not defined in R. %s" % (self.varname, hint)
