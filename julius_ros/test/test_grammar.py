#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import os
import rospkg
import rospy
import unittest

from julius_ros.utils import make_grammar_from_rules
from julius_ros.utils import make_voca_from_categories
from julius_ros.utils import make_dfa
from julius_ros.utils import load_grammar


PKG = rospkg.RosPack().get_path("julius_ros")


class TestGrammar(unittest.TestCase):
    def test_load_grammar(self):
        self.assertTrue(os.path.join(PKG, "data", "udon.grammar"))
        self.assertTrue(os.path.join(PKG, "data", "udon.voca"))
        g = load_grammar(os.path.join(PKG, "data"), "udon")
        self.assertIsNotNone(g)

    def test_grammar(self):
        g = load_grammar(os.path.join(PKG, "data"), "udon")
        gram = make_grammar_from_rules(g.rules)
        self.assertIsNotNone(gram)

    def test_voca(self):
        g = load_grammar(os.path.join(PKG, "data"), "udon")
        voca = make_voca_from_categories(g.categories, g.vocabularies)
        self.assertIsNotNone(voca)

    def test_dfa(self):
        g = load_grammar(os.path.join(PKG, "data"), "udon")
        gram = make_grammar_from_rules(g.rules)
        voca = make_voca_from_categories(g.categories, g.vocabularies)
        result = make_dfa(gram, voca)
        self.assertIsNotNone(result)
        self.assertEqual(len(result), 2)


if __name__ == '__main__':
    import rostest
    rospy.init_node("test_grammar")
    rostest.rosrun("julius_ros", "test_grammar", TestGrammar)
