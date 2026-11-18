#!/usr/bin/env python

import numpy as np

import rospy
from bl_force_torque_sensor_ros.msg import AmplifierOutputRawArray
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32


class SensorDriverRosserial(object):

    def __init__(self):
        self.calib_matrix = rospy.get_param('~calib_matrix', None)
        if self.calib_matrix is None:
            rospy.logfatal(
                'Please specify calibration matrix of your sensor')
            return
        self.calib_matrix = np.array(self.calib_matrix)

        self.sensor_type = rospy.get_param(
            '~sensor_type', 'NANO2.5/2-A')
        if self.sensor_type == 'NANO1.2/1-A':
            self.n_per_count = 4.9e-3  # N/count
            self.nm_per_count = 4.9e-5  # Nm/count
        elif self.sensor_type == 'NANO2.5/2-A':
            self.n_per_count = 9.8e-3
            self.nm_per_count = 9.8e-5
        elif self.sensor_type == 'NANO5/4-A':
            self.n_per_count = 24.5e-3
            self.nm_per_count = 19.6e-5
        else:
            rospy.logfatal(
                'Unimplemented sensor type: %s', self.sensor_type)
            return

        self.amplifier_type = rospy.get_param(
            '~amplifier_type', 'RMD')
        if self.amplifier_type == 'RMD' or self.amplifier_type == 'RMS':
            self.amp_effective_range_max = 5.0
        elif self.amplifier_type == 'RMSH':
            self.amp_effective_range_max = 2.5
        else:
            rospy.logfatal(
                'Unimplemented amplifier type: %s', self.amplifier_type)
            return

        self.adc_type = rospy.get_param(
            '~adc_type', 'ADS131M04')
        if self.adc_type == 'ADS131M04':
            self.voltage_divider = (10.0 + 1.1) / 1.1
            self.adc_range = 2.4  # From -FSR to +FSR
            self.adc_resolution = 24  # Number of bits
        else:
            rospy.logfatal(
                'Unimplemented ADC type: %s', self.adc_type)
            return

        self.sensor_frame_id = rospy.get_param(
            '~sensor_frame_id', 'bl_force_torque_sensor')

        self.rate = rospy.get_param('~rate', -1)  # Hz
        if self.rate <= 0:
            self.loop_duration = 0
        else:
            self.loop_duration = 1.0 / self.rate  # sec
        self.duration_pub = rospy.Publisher(
            '~parameter_setter/loop_duration',
            Float32,
            queue_size=1,
            latch=True
        )
        self.duration_pub.publish(Float32(self.loop_duration))

        self.wrench_pub = rospy.Publisher(
            '~output/wrench', WrenchStamped, queue_size=1)
        self.input_sub = rospy.Subscriber(
            '~input', AmplifierOutputRawArray, self.callback)

    def callback(self, in_msg):
        stamp = rospy.Time.now()  # Get timestamp ASAP after message arrival

        # Convert ADC count of amplifier output to force/torque
        v_raw = np.array(in_msg.array)
        ## Based on ADS131M04 datasheet Revision C p. 38
        v_volt = (v_raw *
                  (float(self.adc_range) / (2 ** self.adc_resolution)) *
                  self.voltage_divider)
        ## Based on NANO sensor manual Rev3.1 p. 8
        v = 2048 * v_volt / self.amp_effective_range_max
        ## Based on NANO sensor manual Rev3.1 p. 9
        f = np.dot(self.calib_matrix, v)
        real_force = f[:3] * self.n_per_count
        real_torque = f[3:] * self.nm_per_count

        # Publish force/torque as WrenchStamped
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = stamp
        wrench_msg.header.frame_id = self.sensor_frame_id
        wrench_msg.wrench.force.x = real_force[0]
        wrench_msg.wrench.force.y = real_force[1]
        wrench_msg.wrench.force.z = real_force[2]
        wrench_msg.wrench.torque.x = real_torque[0]
        wrench_msg.wrench.torque.y = real_torque[1]
        wrench_msg.wrench.torque.z = real_torque[2]
        self.wrench_pub.publish(wrench_msg)


if __name__ == '__main__':
    rospy.init_node('bl_force_torque_sensor_driver')
    app = SensorDriverRosserial()
    rospy.spin()
