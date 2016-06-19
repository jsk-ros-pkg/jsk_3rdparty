#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from . import helper
from .exception import *
import numpy as np
import pandas as pd
import os


class DiscreteNode(object):
    def __init__(self, name, values, parents, children, cpt):
        self.name = name
        self.parents = self._check_val(parents)
        self.values = self._check_val(values)
        self.children = self._check_val(children)
        self.cpt = self._check_val(cpt)
    def __repr__(self):
        return "DiscreteNode: {name} | {parents}".format(name=self.name,
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

class DiscreteBayesianNetwork(helper.RObject):
    def __init__(self, nodes=None, data=None, debug=True, **kwargs):
        super(DiscreteBayesianNetwork, self).__init__(debug)
        self.r('library(bnlearn)')
        if nodes is not None:
            self.add_nodes(nodes)
            self.update_cpts()
        elif data is not None:
            try:
                method=kwargs["method"]
            except:
                method="gs"
            self.estimate_network(data, method, **kwargs)
    def ensure(self, var):
        try:
            self.r[var]
        except:
            raise ValueNotDefinedException(var)
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
            raw_input("hoge")
            self.rcmd("dev.off()")
        self.rcmd("fit <- bn.fit(graph, data)")
        self._parse_graph()
        return self.r["graph"]
    def _parse_graph(self):
        nodenames = self.r["nodes(fit)"]
        self.nodes = []
        for name in nodenames:
            n_dict = self.r["fit$%s" % name]
            val = self.r["rownames(fit$%s$prob)" % name]
            parents = n_dict["parents"]
            children = n_dict["children"]
            cpt = n_dict["prob"]
            self.nodes += [DiscreteNode(name, val, parents, children, cpt)]
    def sample(self, n):
        self.ensure("fit")
        self.r("sample.result <- rbn(fit, %d)" % n)
        return self.r["sample.result"]
    def add_nodes(self, nodes):
        self.nodes = nodes
        self.r["nodes"] = [n.name for n in nodes]
        rcmd = 'graph <- model2network("%s")' % self.model_string()
        self.rcmd(rcmd)
    def update_cpts(self):
        self.ensure("graph")
        rcmd = 'fit <- custom.fit(graph, dist = list('
        need_updated = False
        for n in self.nodes:
            if len(n.cpt) == 0:
                continue
            need_updated = True
            varname = "cpt." + n.name
            self.r[varname] = n.cpt
            dim = 'dim(%s) = c(' % varname
            if len(n.parents) == 0 or n.parents is None:
                dim += '1, %d)' % len(n.values)
            else:
                dim += '%d, ' % len(n.values)
                parents = [n1 for n1 in self.nodes if n1.name in n.parents]
                dim += ', '.join([str(len(p.values)) for p in parents])
                dim += ')'
            self.rcmd(dim)
            dimnames = 'dimnames(%s) = %s' % (varname, self._dimnames_str(n))
            self.rcmd(dimnames)
            rcmd += n.name + " = " + varname
            if n != self.nodes[-1]:
                rcmd += ", "
        rcmd += "))"
        if need_updated:
            self.rcmd(rcmd)
    def _dimnames_str(self, node):
        s = "list("
        if len(node.parents) == 0 or node.parents is None:
            s += 'NULL, c(' + ', '.join(['"%s"' % v for v in node.values]) + '))'
        else:
            parent_nodes = [n for n in self.nodes if n.name in node.parents]
            nodes = [node] + parent_nodes
            for n in nodes:
                s += '"%s" = c(' % n.name
                s += ', '.join(['"%s"' % v for v in n.values])
                s += ')'
                if n != nodes[-1]:
                    s += ', '
            s += ')'
        return s
    def model_string(self):
        s = ""
        for n in self.nodes:
            s += "[" + n.name
            if len(n.parents) > 0:
                s += "|" + ":".join(n.parents)
            s += "]"
        return s
    def print_cprob(self, node_name):
        print self.rcmd('fit$%s' % node_name)
    def fit(self, data):
        self.ensure("graph")
        self.r.assign('data', data)
        self.rcmd('fit <- bn.fit(graph, data)')
        return self.r["fit"]
    def query(self, evidences, q):
        self.ensure("graph")
        self.ensure("fit")
        if type(q) is list:
            rcmd = "query.result <- cpquery(fit, "
            qexp = ' & '.join(['(%s == "%s")' % tuple(e) for e in q])
            rcmd += "eval(parse(text='%s')), " % qexp
        else:
            rcmd = 'query.result <- table(cpdist(fit, "%s", ' % q
        rcmd += " eval(parse(text='"
        rcmd += " & ".join(['(%s == "%s")' % tuple(e) for e in evidences])
        if type(q) is list:
            rcmd += "')))"
        else:
            rcmd += "'))))"
        self.rcmd(rcmd)
        res = self.r["query.result"]
        if type(q) is not list:
            n_sum = sum(res)
            res = map(lambda x: x * 1.0 / n_sum, res)
        return res
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
