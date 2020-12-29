#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from __future__ import print_function
import os
import subprocess
import time
import sys
import unittest
from optparse import OptionParser
import rospy


class TestDownward(unittest.TestCase):
    def test_plan(self):
        self.assertTrue(os.path.isfile(DOMAIN))
        domain = DOMAIN
        self.assertTrue(os.path.isfile(PROBLEM))
        problem = PROBLEM
        config = "ipc seq-sat-fd-autotune-1"
        self.assertIsInstance(PLAN, str)
        plan_path = PLAN
        cmd = "rosrun downward plan %s %s %s --plan-file %s" % (domain,
                                                                problem,
                                                                config,
                                                                plan_path)
        print("EXECUTING: ", cmd, file=sys.stderr)
        status = 0
        try:
            output = subprocess.check_output(cmd, shell=True)
        except subprocess.CalledProcessError as e:
            status = e.returncode
        print("DOWNWARD STATUS: %d, OUTPUT: %s" % (status, output), file=sys.stderr)
        self.assertEqual(status, 0)

        plan_real_path = plan_path + ".1"
        self.assertTrue(os.path.isfile(plan_real_path))

        with open(REF) as f_ref:
            ref_lines = f_ref.readlines()
        with open(plan_real_path) as f_plan:
            plan_lines = f_plan.readlines()
        self.assertSequenceEqual(ref_lines, plan_lines)

if __name__ == '__main__':
    ARGV = rospy.myargv(argv=sys.argv)
    DOMAIN=ARGV[1]
    PROBLEM=ARGV[2]
    PLAN=ARGV[3]
    REF=ARGV[4]
    import rostest
    rostest.rosrun("downward", "test_downward", TestDownward)
