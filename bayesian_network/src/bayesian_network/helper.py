#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import os
from pyper import R
from .exception import *

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

class DAGNode(object):
    def __init__(self, name, parents, children):
        self.name = name
        self.parents = self._check_val(parents)
        self.children = self._check_val(children)
    def __repr__(self):
        return "{cls}: {name} | {parents}".format(cls=self.__class__.__name__,
                                                  name=self.name,
                                                  parents=":".join(self.parents))
    def __str__(self):
        return self.__repr__()
    def _check_val(self, val):
        if val is None or (type(val) is str and len(val) == 0):
            return list()
        elif type(val) is str:
            return [val]
        else:
            return val


class BNLearnObject(RObject):
    def __init__(self, debug=False):
        super(BNLearnObject, self).__init__(debug)
        self.r('library(bnlearn)')
    def ensure(self, var):
        try:
            self.r[var]
        except:
            raise ValueNotDefinedException(var)
    def add_nodes(self, nodes):
        self.nodes = nodes
        self.r["nodes"] = [n.name for n in nodes]
        rcmd = 'graph <- model2network("%s")' % self.model_string()
        self.rcmd(rcmd)
    def model_string(self):
        s = ""
        for n in self.nodes:
            s += "[" + n.name
            if len(n.parents) > 0:
                s += "|" + ":".join(n.parents)
            s += "]"
        return s
    def graph(self):
        self.ensure("graph")
        return self.r["graph"]
    def fitted(self):
        self.ensure("fit")
        return self.r["fit"]
    def estimate_network(self, data, method='gs', whitelist=None, blacklist=None, debug=False):
        self.r["data"] = data
        if whitelist is not None:
            self.r["whitelist"] = pd.DataFrame(np.array(whitelist), columns=["from", "to"])
            self.rcmd('dimnames(whitelist)[2] <- list(c("from", "to"))')
        else:
            self.r["whitelist"] = None
        if blacklist is not None:
            self.r["blacklist"] = pd.DataFrame(np.array(blacklist), columns=["from", "to"])
            self.rcmd('dimnames(blacklist)[2] <- list(c("from", "to"))')
        else:
            self.r["blacklist"] = None
        self.r["debug"] = debug
        self.rcmd("graph <- %s(data, blacklist=blacklist, whitelist=whitelist, debug=debug)" % method)
        if debug:
            self.rcmd("X11()")
            self.rcmd("plot(graph)")
            raw_input("waiting for key press")
            self.rcmd("dev.off()")
        self.rcmd("fit <- bn.fit(graph, data)")
        self._parse_graph()
    def _parse_graph(self):
        raise NotImplementedError(self.__class__.__name__ + "._parse_graph() must be implemented")
    def fit(self, data):
        self.ensure("graph")
        self.r.assign('data', data)
        self.rcmd('fit <- bn.fit(graph, data)')
    def plot_node(self, node_name, to_screen=True, to_pdf=None):
        self.ensure("fit")
        if to_screen:
            self.rcmd("X11()")
        self.rcmd("bn.fit.barchart(fit$%s)" % node_name)
        if to_pdf is not None:
            self.rcmd('dev.copy(pdf, file="%s")' % os.path.abspath(to_pdf))
        if to_screen:
            raw_input("press any key")
        self.rcmd("dev.off()")
    def plot(self, node_name=None, to_screen=True, to_pdf=None):
        if node_name is not None:
            self.plot_node(node_name, to_screen, to_pdf)
            return
        self.ensure("graph")
        if to_screen:
            self.rcmd("X11()")
        self.rcmd("plot(graph)")
        if to_pdf is not None:
            self.rcmd('dev.copy(pdf, file="%s")' % os.path.abspath(to_pdf))
        if to_screen:
            raw_input("press any key")
        self.rcmd("dev.off()")
