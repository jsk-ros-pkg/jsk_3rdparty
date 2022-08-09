#!/usr/bin/env python

import rospy

from influxdb_store.transform_logger import TransformLogger


class BaseTransformLogger(TransformLogger):
    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id', 'base_link')
        self.measurement_name = rospy.get_param(
            '~measurement_name', 'base_transform')
        super(BaseTransformLogger, self).__init__()


if __name__ == '__main__':
    rospy.init_node('base_transform_logger')
    logger = BaseTransformLogger()
    rospy.spin()
