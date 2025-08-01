#!/usr/bin/env python

# Copyright (c) 2017, JSK Laboratory
# All rights reserved.
#
# License: BSD
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the authors nor the names of the
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import serial

from and_scale_ros.msg import WeightStamped
import rospy


class EkEwIDriver(object):

    """Read data from EK-i/EW-i scale.
    Data Sheet: https://www.aandd.co.jp/pdf_storage/manual/balance/m_ekew-i.pdf
    This class should also work with other A&D scales supporting A&D standard format
    (e.g., FZ-i/FX-i/FZ-iWP/FX-iWP (https://www.aandd.co.jp/pdf_storage/manual/balance/m_fzi_fxi_fziwp_fxiwp.pdf))
    """

    def __init__(self):
        super(EkEwIDriver, self).__init__()
        port = rospy.get_param('~port', '/dev/ttyUSB0')
        timeout = rospy.get_param('~timeout', None)
        rospy.loginfo('port=%s', port)
        # EK-i/EW-i series default settings
        baud = rospy.get_param('~baud', 2400)
        self.ser = serial.Serial(
            port, baudrate=baud, bytesize=7, parity=serial.PARITY_EVEN,
            timeout=timeout, writeTimeout=timeout)
        self.pub = rospy.Publisher('~output', WeightStamped, queue_size=1)
        rate = rospy.get_param('~rate', 10)
        self.read_timer = rospy.Timer(rospy.Duration(1. / rate),
                                      self._read_timer_cb)

    def _read_timer_cb(self, event):
        if (self.pub.get_num_connections() == 0):
            return

        try:
            self.ser.write(b'Q\r\n')
        except SerialTimeoutException:
            rospy.logerr('Serial write timeout')
            rospy.signal_shutdown('Serial write timeout')
            return
        data = self.ser.read(17)
        stamp = rospy.Time.now()
        if len(data) != 17:
            rospy.logerr('Serial read timeout')
            rospy.signal_shutdown('Serial read timeout')
            return

        # get scale value
        msg = WeightStamped()
        msg.header.stamp = stamp
        header = data[:2]
        msg.weight.value = float(data[3:12])
        unit = data[12:15]
        if unit == b'  g':
            if header == b'ST':
                # stable
                msg.weight.stable = True
            elif header == b'US':
                # unstable
                msg.weight.stable = False
            elif header == b'OL':
                # scale over
                rospy.logerr('Scale data is over its range')
                return
            elif header == b'QT':
                # number mode
                rospy.logerr('Scale is in number mode')
                return
            else:
                # Unknown header
                rospy.logerr('Unknown header: %s', header)
                return
        else:
            # unit is not g
            rospy.logerr('Unsupported unit: %s', unit)
            return

        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('ekew_i_driver')
    EkEwIDriver()
    rospy.spin()
