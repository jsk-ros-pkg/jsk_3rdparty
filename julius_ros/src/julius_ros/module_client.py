#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import os
import sys
import rospy
from julius_ros.transport import SocketTransport
try:
    from Queue import Queue ## for Python2
except ImportError:
    from queue import Queue ## for Python3
import lxml.etree
import traceback
from xml.sax.saxutils import escape


class ModuleClient(SocketTransport):
    def __init__(self, host, port, max_retry, encoding="utf-8"):
        super(ModuleClient, self).__init__(host, port, max_retry)
        self.encoding = encoding

    def send_command(self, cmds):
        data = os.linesep.join(cmds) + os.linesep
        self.send(data)
        rospy.sleep(0.05)

    def parse(self, data):
        parsed = data.split("." + os.linesep)
        if len(parsed) < 2:
            raise ValueError("Received data too short")
        parsed_data = [self.parse_xml(d) for d in parsed[:-1]]
        parsed_length = len(data) - len(parsed[-1])
        return parsed_data, parsed_length

    def parse_xml(self, data):
        try:
            if sys.version_info.major < 3:
                data = data.decode(self.encoding) ## Python2 needs to convert to utf-8
            data = self.validate_xml(data)
            xml = lxml.etree.fromstring(data)
            return xml.tag, xml
        except Exception as e:
            raise RuntimeError(e)

    def validate_xml(self, data):
        parsed = data.split('"')
        for i in range(len(parsed))[1::2]:
            parsed[i] = escape(parsed[i])
        return '"'.join(parsed)

if __name__ == '__main__':
    pass
