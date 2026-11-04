#!/usr/bin/env python

import influxdb

from influxdb_store.utils import timestamp_to_influxdb_time

from std_msgs.msg import Float32

try:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
except ImportError:
    # jsk_common < 3.0 has no jsk_ros1_ros2_compat, so this is ROS1 only:
    # forward ros_compat.xxx(node, ...) to rospy.xxx(...)
    import rospy
    class ros_compat(object):
        create_subscription = staticmethod(lambda node, *args, **kwargs: rospy.Subscriber(*args, **kwargs))
        get_param = staticmethod(lambda node, *args, **kwargs: rospy.get_param(*args, **kwargs))
        init_node = staticmethod(lambda name, **kwargs: rospy.init_node(name, **kwargs))
        spin = staticmethod(lambda node: rospy.spin())
        logerr = staticmethod(lambda node, *args, **kwargs: rospy.logerr(*args, **kwargs))
        stamp_now = staticmethod(lambda node: rospy.Time.now())


class NetworkStatesLogger(object):
    def __init__(self, node=None):
        self.node = node
        host = ros_compat.get_param(node, '~host', 'localhost')
        port = ros_compat.get_param(node, '~port', 8086)
        database = ros_compat.get_param(node, '~database', 'test')
        self.client = influxdb.InfluxDBClient(
            host=host, port=port, database=database)
        self.client.create_database(database)
        self.sub_transmit = ros_compat.create_subscription(
            node, '~input/transmit', Float32, self._transmit_cb, queue_size=10)
        self.sub_receive = ros_compat.create_subscription(
            node, '~input/receive', Float32, self._receive_cb, queue_size=10)

    def _transmit_cb(self, msg):
        transmit_time = timestamp_to_influxdb_time(ros_compat.stamp_now(self.node))
        transmit_bps = msg.data
        query = [{
            "measurement": "network_states",
            "tags": {
                "type": "transmit"
            },
            "time": transmit_time,
            "fields": {
                "bps": transmit_bps,
            }
        }]
        try:
            self.client.write_points(query, time_precision='ms')
        except influxdb.exceptions.InfluxDBServerError as e:
            ros_compat.logerr(self.node, "InfluxDB error: {}".format(e))

    def _receive_cb(self, msg):
        receive_time = timestamp_to_influxdb_time(ros_compat.stamp_now(self.node))
        receive_bps = msg.data
        query = [{
            "measurement": "network_states",
            "tags": {
                "type": "receive"
            },
            "time": receive_time,
            "fields": {
                "bps": receive_bps,
            }
        }]
        try:
            self.client.write_points(query, time_precision='ms')
        except influxdb.exceptions.InfluxDBServerError as e:
            ros_compat.logerr(self.node, "InfluxDB error: {}".format(e))


if __name__ == '__main__':
    node = ros_compat.init_node('network_states_logger')
    logger = NetworkStatesLogger(node)
    ros_compat.spin(node)
