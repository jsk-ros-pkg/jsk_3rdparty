#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from . import helper
from .exception import *


class DiscreteNode(helper.DAGNode):
    def __init__(self, name, values, parents, children, cpt):
        super(DiscreteNode, self).__init__(name, parents, children)
        self.values = self._check_val(values)
        self.cpt = self._check_val(cpt)

class DiscreteBayesianNetwork(helper.BNLearnObject):
    def __init__(self, nodes=None, data=None, debug=False, **kwargs):
        super(DiscreteBayesianNetwork, self).__init__(debug)
        if nodes is not None:
            self.add_nodes(nodes)
            self.update_cpts()
        elif data is not None:
            try:
                method=kwargs["method"]
            except:
                method="gs"
            self.estimate_network(data, method, **kwargs)
    def _parse_graph(self):
        self.ensure("fit")
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
    def print_cprob(self, node_name):
        print self.rcmd('fit$%s' % node_name)
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
