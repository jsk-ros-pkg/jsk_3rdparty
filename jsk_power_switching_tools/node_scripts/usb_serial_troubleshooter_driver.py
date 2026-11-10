#!/usr/bin/env python

from jsk_power_switching_tools.power_switching_driver_base import PowerSwitchingDriverBase
from jsk_power_switching_tools.power_switching_driver_base import PowerSwitchingDriverBaseCompat
from jsk_power_switching_tools.power_switching_driver_base import ros_compat
if ros_compat is None:  # jsk_common < 3.0 (ROS1 only): use the original rospy code
    import rospy
import serial


class UsbSerialTroubleshooterDriver(PowerSwitchingDriverBase):

    def __init__(self):
        port = rospy.get_param('~port', '/dev/ttyACM0')
        ser_timeout = rospy.get_param('~serial_timeout', None)

        rospy.loginfo('[{}] Port: {}'.format(rospy.get_name(), port))
        # USB-Serial troubleshooter default settings
        self.ser = serial.Serial(
            port,
            baudrate=115200,
            timeout=ser_timeout,
            writeTimeout=ser_timeout,
        )

        super(UsbSerialTroubleshooterDriver, self).__init__()

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


class UsbSerialTroubleshooterDriverCompat(PowerSwitchingDriverBaseCompat):
    """UsbSerialTroubleshooterDriver written with jsk_ros1_ros2_compat, for both ROS1 and ROS2"""

    def __init__(self, node):
        self.node = node
        port = ros_compat.get_param(self.node, '~port', '/dev/ttyACM0')
        ser_timeout = ros_compat.get_param(self.node, '~serial_timeout', None)

        ros_compat.loginfo(self.node, '[{}] Port: {}'.format(ros_compat.get_name(self.node), port))
        # USB-Serial troubleshooter default settings
        self.ser = serial.Serial(
            port,
            baudrate=115200,
            timeout=ser_timeout,
            writeTimeout=ser_timeout,
        )

        super(UsbSerialTroubleshooterDriverCompat, self).__init__(node)

    def _pwr(self, is_on):
        try:
            self.ser.write('PW={}\r\n'.format(int(is_on)).encode())
        except (serial.SerialTimeoutException, OSError) as e:
            ros_compat.logerr(self.node,
                '[{}] Serial write failed: {}'.format(ros_compat.get_name(self.node), e))
            ros_compat.signal_shutdown(self.node, 'Serial write failed')
            return False
        try:
            res = self.ser.read(4)
        except OSError as e:
            ros_compat.logerr(self.node,
                '[{}] Serial read failed: {}'.format(ros_compat.get_name(self.node), e))
            ros_compat.signal_shutdown(self.node, 'Serial read failed')
            return False
        if res == b'OK\r\n':
            ros_compat.loginfo(self.node,
                '[{}] Changed power state to {}'.format(
                    ros_compat.get_name(self.node), is_on))
            return True
        else:
            try:
                res += self.ser.read(3)
            except OSError as e:
                ros_compat.logerr(self.node,
                    '[{}] Serial read failed: {}'.format(ros_compat.get_name(self.node), e))
                ros_compat.signal_shutdown(self.node, 'Serial read failed')
            if len(res) != 7:
                ros_compat.logerr(self.node,
                    '[{}] Serial read timeout'.format(ros_compat.get_name(self.node)))
                ros_compat.signal_shutdown(self.node, 'Serial read timeout')
            else:
                ros_compat.logerr(self.node,
                    '[{}] Return value is {}'.format(
                        ros_compat.get_name(self.node), res))
                ros_compat.signal_shutdown(self.node, 'Return value is erroneous')
            return False


if __name__ == '__main__' and ros_compat is None:
    rospy.init_node('usb_serial_troubleshooter_driver')
    app = UsbSerialTroubleshooterDriver()
    rospy.spin()

elif __name__ == '__main__':  # with jsk_ros1_ros2_compat (ROS1/ROS2)
    node = ros_compat.init_node('usb_serial_troubleshooter_driver')
    app = UsbSerialTroubleshooterDriverCompat(node)
    ros_compat.spin(node)
