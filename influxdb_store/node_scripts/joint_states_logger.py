#!/usr/bin/env python

import copy
import influxdb
import sys
import threading
import time

from influxdb_store.utils import timestamp_to_influxdb_time

from sensor_msgs.msg import JointState

try:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
except ImportError:
    # jsk_common < 3.0 has no jsk_ros1_ros2_compat, so this is ROS1 only:
    # forward ros_compat.xxx(node, ...) to rospy.xxx(...)
    import rospy
    class ros_compat(object):
        create_subscription = staticmethod(lambda node, *args, **kwargs: rospy.Subscriber(*args, **kwargs))
        create_timer = staticmethod(lambda node, period_sec, callback: rospy.Timer(rospy.Duration(period_sec), callback))
        get_param = staticmethod(lambda node, *args, **kwargs: rospy.get_param(*args, **kwargs))
        init_node = staticmethod(lambda name, **kwargs: rospy.init_node(name, **kwargs))
        spin = staticmethod(lambda node: rospy.spin())
        logdebug = staticmethod(lambda node, *args, **kwargs: rospy.logdebug(*args, **kwargs))
        logerr = staticmethod(lambda node, *args, **kwargs: rospy.logerr(*args, **kwargs))


class JointStatesLogger(object):
    def __init__(self, node=None):
        self.node = node
        host = ros_compat.get_param(node, '~host', 'localhost')
        port = ros_compat.get_param(node, '~port', 8086)
        database = ros_compat.get_param(node, '~database', 'test')
        self.duration = ros_compat.get_param(node, '~duration', 3.0)
        self.client = influxdb.InfluxDBClient(
            host=host, port=port, database=database)
        self.client.create_database(database)
        self.sub = ros_compat.create_subscription(
            node, '~input', JointState, self._cb, queue_size=300)
        self.timer = ros_compat.create_timer(node, self.duration, self._timer_cb)
        self.lock = threading.Lock()
        self.query = []

    def _cb(self, msg):
        influx_time = timestamp_to_influxdb_time(msg.header.stamp)
        joint_names = msg.name
        position = msg.position
        velocity = msg.velocity
        effort = msg.effort
        with self.lock:
            self.query.append({
                "measurement": "joint_states",
                "tags": {
                    "type": "position"
                },
                "time": influx_time,
                "fields": dict(list(zip(joint_names, position)))
            })
            self.query.append({
                "measurement": "joint_states",
                "tags": {
                    "type": "velocity"
                },
                "time": influx_time,
                "fields": dict(list(zip(joint_names, velocity)))
            })
            self.query.append({
                "measurement": "joint_states",
                "tags": {
                    "type": "effort"
                },
                "time": influx_time,
                "fields": dict(list(zip(joint_names, effort)))
            })

    def _timer_cb(self, event):
        start_time = time.time() * 1000
        with self.lock:
            # no joint_states coming, so stop nodes
            if len(self.query) == 0:
                sys.exit(1)
            query = copy.deepcopy(self.query)
            self.query = []
        end_time = time.time() * 1000
        ros_compat.logdebug(self.node, "copy time: {}ms".format(end_time - start_time))
        ros_compat.logdebug(self.node, "data length: {}".format(len(query)))
        try:
            self.client.write_points(query, time_precision='ms')
        except influxdb.exceptions.InfluxDBServerError as e:
            ros_compat.logerr(self.node, "InfluxDB error: {}".format(e))
        end_time = time.time() * 1000
        ros_compat.logdebug(self.node, "timer cb time: {}ms".format(end_time - start_time))
        if ((end_time - start_time) > (self.duration * 1000)):
            ros_compat.logerr(self.node, "timer cb time exceeds: {} > {}".format(
                end_time - start_time, self.duration * 1000))


if __name__ == '__main__':
    node = ros_compat.init_node('joint_states_logger')
    logger = JointStatesLogger(node)
    ros_compat.spin(node)
