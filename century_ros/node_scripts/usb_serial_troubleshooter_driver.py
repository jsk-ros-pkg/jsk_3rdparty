#!/usr/bin/env python

from std_srvs.srv import SetBool
from std_srvs.srv import SetBoolResponse
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse
import rospy
import serial
import threading


class UsbSerialTroubleshooterDriver(object):

    def __init__(self):
        port = rospy.get_param('~port', '/dev/ttyACM0')
        ser_timeout = rospy.get_param('~serial_timeout', None)
        self.pwr_cyc_iv = rospy.get_param('~power_cycle_interval', 1.0)
        self.is_init_on = rospy.get_param('~init_with_power_on', True)
        self.lock = threading.Lock()

        rospy.loginfo('[{}] Port: {}'.format(rospy.get_name(), port))
        # USB-Serial troubleshooter default settings
        self.ser = serial.Serial(
            port,
            baudrate=115200,
            timeout=ser_timeout,
            writeTimeout=ser_timeout,
        )
        if self.is_init_on:
            self._pwr(True)

        self.pwr_srv = rospy.Service(
            '~power', SetBool, self._pwr_srv_cb)
        self.pwr_cyc_srv = rospy.Service(
            '~power_cycle', Trigger, self._pwr_cyc_srv_cb)

    def _pwr(self, is_on):
        try:
            self.ser.write('PW={}\r\n'.format(int(is_on)).encode())
        except (serial.SerialTimeoutException, OSError) as e:
            rospy.logerr(
                '[{}] Serial write failed: {}'.format(rospy.get_name(), e))
            rospy.signal_shutdown('Serial write failed')
            return False
        try:
            res = self.ser.read(4)
        except OSError as e:
            rospy.logerr(
                '[{}] Serial read failed: {}'.format(rospy.get_name(), e))
            rospy.signal_shutdown('Serial read failed')
            return False
        if res == b'OK\r\n':
            rospy.loginfo(
                '[{}] Changed power state to {}'.format(
                    rospy.get_name(), is_on))
            return True
        else:
            try:
                res += self.ser.read(3)
            except OSError as e:
                rospy.logerr(
                    '[{}] Serial read failed: {}'.format(rospy.get_name(), e))
                rospy.signal_shutdown('Serial read failed')
            if len(res) != 7:
                rospy.logerr(
                    '[{}] Serial read timeout'.format(rospy.get_name()))
                rospy.signal_shutdown('Serial read timeout')
            else:
                rospy.logerr(
                    '[{}] Return value is {}'.format(
                        rospy.get_name(), res))
                rospy.signal_shutdown('Return value is erroneous')
            return False

    def _pwr_srv_cb(self, req):
        with self.lock:
            res = self._pwr(req.data)
        return SetBoolResponse(success=res)

    def _pwr_cyc_srv_cb(self, req):
        with self.lock:
            res = self._pwr(False)
            if res:
                rospy.sleep(self.pwr_cyc_iv)
                res = self._pwr(True)
        return TriggerResponse(success=res)


if __name__ == '__main__':
    rospy.init_node('usb_serial_troubleshooter_driver')
    app = UsbSerialTroubleshooterDriver()
    rospy.spin()
