#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from __future__ import print_function
import os
import rospkg
import sys
import subprocess
import unittest

PKGNAME = "darknet"
PKGPATH = rospkg.RosPack().get_path(PKGNAME)
DATA_CFG_PATH = os.path.join(PKGPATH, "cfg", "coco.data")
CFG_PATH = os.path.join(PKGPATH, "cfg", "yolo.cfg")
WEIGHT_PATH = os.path.join(PKGPATH, "weights", "yolo.weights")
IMG_PATH = os.path.join(PKGPATH, "data", "dog.jpg")


class TestDarknet(unittest.TestCase):
    def test_darknet(self):
        self.assertTrue(os.path.isfile(DATA_CFG_PATH), "%s not found" % DATA_CFG_PATH)
        self.assertTrue(os.path.isfile(CFG_PATH), "%s not found" % CFG_PATH)
        self.assertTrue(os.path.isfile(WEIGHT_PATH), "%s not found" % WEIGHT_PATH)
        self.assertTrue(os.path.isfile(IMG_PATH), "%s not found" % IMG_PATH)
        cmd = "DISPLAY= rosrun darknet darknet detector test %s %s %s %s" % (DATA_CFG_PATH, CFG_PATH, WEIGHT_PATH, IMG_PATH)
        print("EXECUTING: ", cmd, file=sys.stderr)

        p = subprocess.Popen(cmd, shell=True, cwd=PKGPATH,
                             stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE)
        out_str, err_str = p.communicate()
        status = p.returncode
        print("DARKNET OUTPUT: %s, ERROR: %s" % (out_str, err_str), file=sys.stderr)

        self.assertTrue("Done!" in err_str, "failed loading weight")
        self.assertTrue("dog" in out_str, "failed detection")
        RESULT_PATH = os.path.join(PKGPATH, "predictions.jpg")
        if os.path.exists(RESULT_PATH):
            os.remove(RESULT_PATH)
