from std_srvs.srv import SetBool
from std_srvs.srv import Trigger

try:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
except ImportError:
    ros_compat = None  # jsk_common < 3.0 (ROS1 only): use the original rospy code

if ros_compat is None:
    from std_srvs.srv import SetBoolResponse
    from std_srvs.srv import TriggerResponse
    import rospy
else:
    SetBoolResponse = ros_compat.get_response_class(SetBool)
    TriggerResponse = ros_compat.get_response_class(Trigger)
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


class PowerSwitchingDriverBaseCompat(PowerSwitchingDriverBase):
    """PowerSwitchingDriverBase written with jsk_ros1_ros2_compat, for both
    ROS1 and ROS2: `node` is None on ROS1 and the rclpy node on ROS2"""

    def __init__(self, node):
        self.node = node
        self.pwr_cyc_iv = ros_compat.get_param(node, '~power_cycle_interval', 1.0)
        self.is_init_set = ros_compat.get_param(node, '~init_with_power_set', True)
        self.is_init_on = ros_compat.get_param(node, '~init_with_power_on', True)
        self.lock = threading.Lock()

        if self.is_init_set:
            if self.is_init_on:
                self._pwr(True)
            else:
                self._pwr(False)

        # ros_compat's service callbacks take (request, response)
        self.pwr_srv = ros_compat.create_service(
            node, '~power', SetBool,
            lambda req, res: self._pwr_srv_cb(req))
        self.pwr_cyc_srv = ros_compat.create_service(
            node, '~power_cycle', Trigger,
            lambda req, res: self._pwr_cyc_srv_cb(req))

    def _pwr_cyc_srv_cb(self, req):
        with self.lock:
            res = self._pwr(False)
            if res:
                ros_compat.sleep(self.node, self.pwr_cyc_iv)
                res = self._pwr(True)
        return TriggerResponse(success=res)
