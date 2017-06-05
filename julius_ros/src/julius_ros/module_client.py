#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import os
import rospy
from .transport import SocketTransport
from Queue import Queue
import lxml.etree
import traceback


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
            # received data is too short
            raise ValueError
        parsed_data = [self.parse_xml(d) for d in parsed[:-1]]
        parsed_length = len(data) - len(parsed[-1])
        return parsed_data, parsed_length

    def parse_xml(self, data):
        xml = lxml.etree.fromstring(data.decode(self.encoding))
        return xml.tag, xml

if __name__ == '__main__':
    pass
