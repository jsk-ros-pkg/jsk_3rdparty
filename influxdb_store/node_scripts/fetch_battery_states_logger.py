#!/usr/bin/env python

import influxdb
import rospy

from influxdb_store.utils import timestamp_to_influxdb_time

from power_msgs.msg import BatteryState


class FetchBatteryStatesLogger(object):
    def __init__(self):
        host = rospy.get_param('~host', 'localhost')
        port = rospy.get_param('~port', 8086)
        database = rospy.get_param('~database', 'test')
        self.client = influxdb.InfluxDBClient(
            host=host, port=port, database=database)
        self.client.create_database(database)
        self.sub = rospy.Subscriber(
            '~input', BatteryState, self._cb, queue_size=10)

    def _cb(self, msg):
        time = timestamp_to_influxdb_time(rospy.Time.now())
        battery_name = msg.name
        charge_percent = msg.charge_level
        query = [{
            "measurement": "battery_states",
            "tags": {
                "battery_name": battery_name
            },
            "time": time,
            "fields": {
                "charge_percent": charge_percent,
            }
        }]
        try:
            self.client.write_points(query, time_precision='ms')
        except influxdb.exceptions.InfluxDBServerError as e:
            rospy.logerr("InfluxDB error: {}".format(e))


if __name__ == '__main__':
    rospy.init_node('fetch_battery_states_logger')
    logger = FetchBatteryStatesLogger()
    rospy.spin()
