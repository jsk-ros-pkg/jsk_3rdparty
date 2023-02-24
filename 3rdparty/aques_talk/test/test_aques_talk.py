#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import os
import subprocess
import sys
import tempfile
import unittest
import rospy


class TestAquestalk(unittest.TestCase):

    def assertIsFile(self, path):
        # https://stackoverflow.com/questions/59121161/python-unittest-how-to-assert-the-existence-of-a-file-or-folder-and-print-the-p
        if not os.path.exists(path):
            raise AssertionError("File does not exist: %s" % str(path))

    def test_text2wave(self):
        input_file = '/tmp/hello.txt'
        output_file = os.path.join(tempfile.mkdtemp(), 'hello.wav')
        with open(input_file, 'w') as f:
            f.write('Hello World!')
        cmd = "rosrun aques_talk text2wave -o %s %s" % (output_file, input_file)
        print("EXECUTING: ", cmd, file=sys.stderr)
        output = ''
        status = 0
        try:
            output = subprocess.check_output(cmd, shell=True)
        except subprocess.CalledProcessError as e:
            status = e.returncode
        print("AQUESTALK STATUS: %d, OUTPUT: %s" % (status, output), file=sys.stderr)
        self.assertEqual(status, 0)

        # check output file
        self.assertIsFile(output_file)


if __name__ == '__main__':
    import rostest
    rostest.rosrun("aquestalk", "test_aquestalk", TestAquestalk)
