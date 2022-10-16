#!/usr/bin/env python

import influxdb
import rospy

from influxdb_store.utils import timestamp_to_influxdb_time

from pr2_msgs.msg import BatteryServer2


class PR2BatteryStatesLogger(object):
    def __init__(self):
        host = rospy.get_param('~host', 'localhost')
        port = rospy.get_param('~port', 8086)
        database = rospy.get_param('~database', 'test')
        self.client = influxdb.InfluxDBClient(
            host=host, port=port, database=database)
        self.client.create_database(database)
        self.sub = rospy.Subscriber(
            '~input', BatteryServer2, self._cb, queue_size=10)

    def _cb(self, msg):
        time = timestamp_to_influxdb_time(msg.header.stamp)
        battery_id = msg.id
        battery0_temp = msg.battery[0].battery_register[8] / 10.0 - 273.15
        battery1_temp = msg.battery[1].battery_register[8] / 10.0 - 273.15
        battery2_temp = msg.battery[2].battery_register[8] / 10.0 - 273.15
        battery3_temp = msg.battery[3].battery_register[8] / 10.0 - 273.15
        battery0_voltage = msg.battery[0].battery_register[9] / 1000.0
        battery1_voltage = msg.battery[1].battery_register[9] / 1000.0
        battery2_voltage = msg.battery[2].battery_register[9] / 1000.0
        battery3_voltage = msg.battery[3].battery_register[9] / 1000.0
        battery0_current = msg.battery[0].battery_register[11] / 1000.0
        battery1_current = msg.battery[1].battery_register[11] / 1000.0
        battery2_current = msg.battery[2].battery_register[11] / 1000.0
        battery3_current = msg.battery[3].battery_register[11] / 1000.0
        battery0_charge = msg.battery[0].battery_register[13]
        battery1_charge = msg.battery[1].battery_register[13]
        battery2_charge = msg.battery[2].battery_register[13]
        battery3_charge = msg.battery[3].battery_register[13]
        average_temp = (battery0_temp + battery1_temp
                        + battery2_temp + battery3_temp) / 4.0
        average_voltage = (battery0_voltage + battery1_voltage
                           + battery2_voltage + battery3_voltage) / 4.0
        average_current = (battery0_current + battery1_current
                           + battery2_current + battery3_current) / 4.0
        average_charge = msg.average_charge / 100.0
        query = [{
            "measurement": "battery_states",
            "tags": {
                "battery_id": battery_id
            },
            "time": time,
            "fields": {
                "temperature": average_temp,
                "voltage": average_voltage,
                "current": average_current,
                "charge_percent": average_charge,
                "battery0_temperature": battery0_temp,
                "battery1_temperature": battery1_temp,
                "battery2_temperature": battery2_temp,
                "battery3_temperature": battery3_temp,
                "battery0_voltage": battery0_voltage,
                "battery1_voltage": battery1_voltage,
                "battery2_voltage": battery2_voltage,
                "battery3_voltage": battery3_voltage,
                "battery0_current": battery0_current,
                "battery1_current": battery1_current,
                "battery2_current": battery2_current,
                "battery3_current": battery3_current,
                "battery0_charge_percent": battery0_charge,
                "battery1_charge_percent": battery1_charge,
                "battery2_charge_percent": battery2_charge,
                "battery3_charge_percent": battery3_charge,
            }
        }]
        if len(query) > 0:
            try:
                self.client.write_points(query, time_precision='ms')
            except influxdb.exceptions.InfluxDBServerError as e:
                rospy.logerr("InfluxDB error: {}".format(e))


if __name__ == '__main__':
    rospy.init_node('pr2_battery_states_logger')
    logger = PR2BatteryStatesLogger()
    rospy.spin()
