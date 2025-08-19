#!/usr/bin/env python

import serial
import threading

import rospy
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse


class SensorDriver(object):

    def __init__(self):
        super(SensorDriver, self).__init__()
        port = rospy.get_param('~port', '/dev/ttyUSB0')
        baud = rospy.get_param('~baud', 115200)
        self.n_per_hz = rospy.get_param('~n_per_hz', 0.0000286)
        self.calib_sample_num = rospy.get_param(
            '~offset_calib_sample_number', 300)
        self.sensor_frame_id = rospy.get_param(
            '~sensor_frame_id', 'qcr_force_sensor')
        rospy.loginfo('port=%s, baud=%s', port, baud)
        self.ser = serial.Serial(port, baudrate=baud)
        self.offset = 0.0
        self.last_raw_val = None
        self.last_stamp = None
        self.lock = threading.Lock()
        self.calib_srv = rospy.Service(
            '~calibrate_offset', Empty, self.calib_offset)
        self.wrench_pub = rospy.Publisher(
            '~output/wrench', WrenchStamped, queue_size=1)

    def calib_offset(self, req):
        rospy.loginfo(
            '[{}] offset calibration started...'.format(
                rospy.get_name()))

        while self.last_stamp is None:
            rospy.sleep(0.00001)

        with self.lock:
            last_stamp_in_vals = self.last_stamp
            vals = [self.last_raw_val]
        for i in range(self.calib_sample_num):
            while self.last_stamp <= last_stamp_in_vals:
                rospy.sleep(0.00001)

            with self.lock:
                last_stamp_in_vals = self.last_stamp
                vals.append(self.last_raw_val)

        with self.lock:
            self.offset = sum(vals) / float(len(vals))
        rospy.loginfo(
            '[{}] offset calibration finished. Offset: {}'.format(
                rospy.get_name(), self.offset))

        return EmptyResponse()

    def run(self):
        self.ser.readline()  # Flush first data because it is hardly complete.
        # Here we assume this process is just after initialization of ser.
        # Otherwise data in input buffer become too many and collapse.
        # Flushing them is difficult:
        # https://github.com/pyserial/pyserial/issues/344
        while not rospy.is_shutdown():
            raw_val = float(self.ser.readline())
            with self.lock:
                self.last_raw_val = raw_val
                self.last_stamp = rospy.Time.now()  # Get timestamp ASAP after message arrival

                force = (raw_val - self.offset) * self.n_per_hz

            # Publish force as WrenchStamped
            wrench_msg = WrenchStamped()
            wrench_msg.header.stamp = self.last_stamp
            wrench_msg.header.frame_id = self.sensor_frame_id
            wrench_msg.wrench.force.x = force
            wrench_msg.wrench.force.y = 0
            wrench_msg.wrench.force.z = 0
            wrench_msg.wrench.torque.x = 0
            wrench_msg.wrench.torque.y = 0
            wrench_msg.wrench.torque.z = 0
            self.wrench_pub.publish(wrench_msg)


if __name__ == '__main__':
    rospy.init_node('qcr_force_sensor_driver')
    app = SensorDriver()
    app.run()
