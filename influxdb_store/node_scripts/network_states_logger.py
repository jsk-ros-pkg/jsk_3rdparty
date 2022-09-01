#!/usr/bin/env python

import influxdb
import rospy

from influxdb_store.utils import timestamp_to_influxdb_time

from std_msgs.msg import Float32


class NetworkStatesLogger(object):
    def __init__(self):
        host = rospy.get_param('~host', 'localhost')
        port = rospy.get_param('~port', 8086)
        database = rospy.get_param('~database', 'test')
        self.client = influxdb.InfluxDBClient(
            host=host, port=port, database=database)
        self.client.create_database(database)
        self.sub_transmit = rospy.Subscriber(
            '~input/transmit', Float32, self._transmit_cb, queue_size=10)
        self.sub_receive = rospy.Subscriber(
            '~input/receive', Float32, self._receive_cb, queue_size=10)

    def _transmit_cb(self, msg):
        transmit_time = timestamp_to_influxdb_time(rospy.Time.now())
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
            rospy.logerr("InfluxDB error: {}".format(e))

    def _receive_cb(self, msg):
        receive_time = timestamp_to_influxdb_time(rospy.Time.now())
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
            rospy.logerr("InfluxDB error: {}".format(e))


if __name__ == '__main__':
    rospy.init_node('network_states_logger')
    logger = NetworkStatesLogger()
    rospy.spin()
