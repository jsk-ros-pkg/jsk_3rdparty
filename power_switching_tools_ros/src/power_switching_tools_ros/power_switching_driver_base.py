from std_srvs.srv import SetBool
from std_srvs.srv import SetBoolResponse
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse
import rospy
import threading


class PowerSwitchingDriverBase(object):

    def __init__(self):
        self.pwr_cyc_iv = rospy.get_param('~power_cycle_interval', 1.0)
        self.is_init_set = rospy.get_param('~init_with_power_set', True)
        self.is_init_on = rospy.get_param('~init_with_power_on', True)
        self.lock = threading.Lock()

        if self.is_init_set:
            if self.is_init_on:
                self._pwr(True)
            else:
                self._pwr(False)

        self.pwr_srv = rospy.Service(
            '~power', SetBool, self._pwr_srv_cb)
        self.pwr_cyc_srv = rospy.Service(
            '~power_cycle', Trigger, self._pwr_cyc_srv_cb)

    def _pwr(self, is_on):
        raise NotImplementedError

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

