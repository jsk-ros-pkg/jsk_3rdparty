#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import os
import rospkg
import rospy
import subprocess
import tempfile

_PKG=rospkg.RosPack().get_path("julius_ros")
_YOMI2VOCA=os.path.join(_PKG, "scripts", "yomi2voca.pl")

def make_phonemes_from_words(words):
    cmd = os.linesep.join(["%s %s" % (w, w) for w in words]) + os.linesep
    p = subprocess.Popen(["perl", _YOMI2VOCA],
                         stdin=subprocess.PIPE,
                         stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE)
    result, error = p.communicate(cmd)
    if error:
        rospy.logerr("Error: %s" % error)
        return None
    result = result.split(os.linesep)[:-1]
    result = [r.split("\t")[1] for r in result]
    return result

if __name__ == '__main__':
    result = make_phonemes_from_words(["うどん", "そば"])
    assert result[0] == "u d o N"
    assert result[1] == "s o b a"
