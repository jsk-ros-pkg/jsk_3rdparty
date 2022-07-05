#!/usr/bin/env python

import rospy

from influxdb_store.transform_logger import TransformLogger


class MapTransformLogger(TransformLogger):
    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.measurement_name = rospy.get_param(
            '~measurement_name', 'map_transform')
        super(MapTransformLogger, self).__init__()


if __name__ == '__main__':
    rospy.init_node('map_transform_logger')
    logger = MapTransformLogger()
    rospy.spin()
