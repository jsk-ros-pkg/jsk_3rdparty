#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


class NoStructureException(Exception):
    def __str__(self):
        return 'No structure found in graph. try add_nodes method'

class NoNetworkException(Exception):
    def __str__(self):
        return "No network found in graph. try fit method"
