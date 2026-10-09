#!/usr/bin/env python

from std_srvs.srv import Trigger
import rospy


class AutoPowerCycle(object):

    def __init__(self):
        self.expt_hz = rospy.get_param(
            '~monitored_topic_expected_hz', 10)
        self.allowed_dly = rospy.get_param(
            '~monitored_topic_allowed_delay_cycles', 2.0)
        self.respawn_dly = rospy.get_param(
            '~monitored_topic_respawn_delay', 10.0)
        self.is_init_cyc = rospy.get_param('~init_with_power_cycle', True)
        self.is_topic_sub = False
        self.limit_tm = None

        self.mon_topic_sub = rospy.Subscriber(
            '~monitored_topic',
            rospy.AnyMsg,
            self._mon_topic_cb,
        )
        srv_name = '~power_cycle'
        rospy.logwarn(
            '[{}] Waiting for power cycle service...'.format(
                rospy.get_name()))
        rospy.wait_for_service(srv_name)
        rospy.loginfo(
            '[{}] Found power cycle service'.format(
                rospy.get_name()))
        self.pwr_cyc_srv = rospy.ServiceProxy(srv_name, Trigger)
        if self.is_init_cyc:
            res = self.pwr_cyc_srv()
        self.monitoring_timer = rospy.Timer(
            rospy.Duration(1.0 / (self.expt_hz * 10)), self._monitoring_cb)

    def _mon_topic_cb(self, msg):
        self.is_topic_sub = True

    def _monitoring_cb(self, timer):
        if self.is_topic_sub:
            self.is_topic_sub = False
            if self.limit_tm is None:
                rospy.loginfo(
                    '[{}] Got first message of '
                    'monitored topic'.format(rospy.get_name()))
            self.limit_tm = rospy.Time.now() + rospy.Duration(
                (1.0 / self.expt_hz) * (1 + self.allowed_dly))
        else:
            if self.limit_tm is None:
                rospy.logwarn_throttle(
                    10,
                    '[{}] First message of monitored topic does not '
                    'come yet, waiting...'.format(rospy.get_name()))
            elif rospy.Time.now() > self.limit_tm:
                rospy.logwarn(
                    '[{}] Monitored topic stops, '
                    'try power cycle...'.format(rospy.get_name()))
                try:
                    res = self.pwr_cyc_srv()
                except rospy.service.ServiceException as e:
                    rospy.logwarn(
                        '[{}] power cycle failed due to '
                        'service call failure, try next time'.format(
                            rospy.get_name()))
                    rospy.logdebug(
                        '[{}] Service call error: {}'.format(
                            rospy.get_name(), e))
                self.limit_tm = rospy.Time.now() + rospy.Duration(
                    self.respawn_dly)


if __name__ == '__main__':
    rospy.init_node('auto_power_cycle')
    app = AutoPowerCycle()
    rospy.spin()
