#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from . import helper

class DiscreteNode(object):
    def __init__(self, name, values, parents, children, cpt):
        self.name = name
        self.parents = parents
        self.values = values
        self.children = children
        self.cpt = cpt

class DiscreteBayesianNetwork(helper.RObject):
    def __init__(self, nodes=None, method="empty"):
        super(DiscreteBayesianNetwork, self).__init__(True)
        self.r('library(bnlearn)')
        self.nodes = nodes
        self.r["nodes"] = [n.name for n in nodes]
        self.r["method"] = method
        rcmd = 'graph = model2network("%s")' % self.model_string()
        self.rcmd(rcmd)
    def update_cpts(self):
        rcmd = 'fit = custom.fit(graph, dist = list('
        need_updated = False
        for n in self.nodes:
            if n.cpt is None:
                continue
            need_updated = True
            varname = "cpt." + n.name
            self.r[varname] = n.cpt
            dim = 'dim(%s) = c(' % varname
            if n.parents is None:
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
        if node.parents is None:
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
            if n.parents:
                s += "|" + ":".join(n.parents)
            s += "]"
        return s
    def estimate_network(self, data, method='gs'):
        self.r["data"] = data
        self.r('graph <- %s(data)' % method)
        return self.r["graph"]
    def fit(self, data):
        self.r["data"] = data
        self.r('fit = bn.fit(graph, data)')
        return self.r["fit"]
    def query(self, evidences, q):
        if q is list:
            rcmd = "query.result = cpquery(fit, "
            qexp = ' & '.join(['(%s == "%s")' % tuple(e) for e in q])
            rcmd += 'eval(parse(text="%s")), ' % qexp
        else:
            rcmd = 'query.result = table(cpdist(fit, "%s", ' % q
        rcmd += " eval(parse(text='"
        rcmd += " & ".join(['(%s == "%s")' % tuple(e) for e in evidences])
        rcmd += "'))))"
        self.rcmd(rcmd)
        return self.r["query.result"]
