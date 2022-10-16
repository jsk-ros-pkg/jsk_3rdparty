import copy
import influxdb
import numpy as np
import rospy
import sys
import threading
import time
import yaml

import tf2_ros

from influxdb_store.utils import timestamp_to_influxdb_time
from tf2_msgs.srv import FrameGraph


class TransformLogger(object):
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.wait_for_service("~tf2_frames")
        self.get_frames = rospy.ServiceProxy("~tf2_frames", FrameGraph)
        self.graph = {}

        trial_count = 0
        while len(self.graph) == 0:
            try:
                if trial_count > 1200:
                    sys.exit(1)
                self.graph = yaml.load(self.get_frames().frame_yaml)
            except Exception as e:
                rospy.logerr_throttle(
                    60.0, 'Failed to get graph: {}'.format(e))
                rospy.sleep(0.1)
                trial_count = trial_count + 1

        host = rospy.get_param('~host', 'localhost')
        port = rospy.get_param('~port', 8086)
        database = rospy.get_param('~database', 'test')
        self.client = influxdb.InfluxDBClient(
            host=host, port=port, database=database)
        self.client.create_database(database)

        self.lock = threading.Lock()
        self.graph_lock = threading.Lock()
        self.query = []

        self.update_duration = 1.0 / rospy.get_param('~update_frequency', 30.0)
        self.update_timer = rospy.Timer(
            rospy.Duration(self.update_duration), self._update_cb)

        self.duration = rospy.get_param('~duration', 3.0)
        self.write_timer = rospy.Timer(
            rospy.Duration(self.duration), self._timer_cb)

        graph_duration = rospy.get_param('~graph_duration', 60.0)
        self.graph_timer = rospy.Timer(
            rospy.Duration(graph_duration), self._graph_cb)

    def _update_cb(self, event):
        # if event.last_real:
        #     timestamp = event.last_real
        # else:
        #     timestamp = event.current_real - self.update_rate
        timestamp = event.current_real
        influx_time = timestamp_to_influxdb_time(timestamp)
        trans_x_fields = {}
        trans_y_fields = {}
        trans_z_fields = {}
        rot_x_fields = {}
        rot_y_fields = {}
        rot_z_fields = {}
        rot_w_fields = {}
        rot_theta_fields = {}

        with self.graph_lock:
            graph = copy.deepcopy(self.graph)

        for child_frame_id, _ in graph.items():
            try:
                transform_stamped = self.tf_buffer.lookup_transform(
                    self.frame_id, child_frame_id, rospy.Time(0))
            except tf2_ros.ExtrapolationException as e:
                rospy.logerr_throttle(
                    60.0, 'tf2_ros.ExtrapolationException: {}'.format(e))
                continue
            except tf2_ros.ConnectivityException as e:
                rospy.logerr_throttle(
                    60.0, 'tf2_ros.ConnectivityException: {}'.format(e))
                continue
            except tf2_ros.LookupException as e:
                rospy.logerr_throttle(
                    60.0, 'tf2_ros.LookupException: {}'.format(e))
                continue
            translation = transform_stamped.transform.translation
            rotation = transform_stamped.transform.rotation
            theta = 2 * np.arctan(
                np.linalg.norm(
                    [rotation.x, rotation.y, rotation.z]) / rotation.w)

            trans_x_fields[child_frame_id] = translation.x
            trans_y_fields[child_frame_id] = translation.y
            trans_z_fields[child_frame_id] = translation.z
            rot_x_fields[child_frame_id] = rotation.x
            rot_y_fields[child_frame_id] = rotation.y
            rot_z_fields[child_frame_id] = rotation.z
            rot_w_fields[child_frame_id] = rotation.w
            rot_theta_fields[child_frame_id] = theta

        with self.lock:
            if len(trans_x_fields) > 0:
                self.query.append({
                    "measurement": self.measurement_name,
                    "tags": {
                        "type": "translation",
                        "field": "x",
                        "frame_id": self.frame_id
                    },
                    "time": influx_time,
                    "fields": trans_x_fields
                })

            if len(trans_y_fields) > 0:
                self.query.append({
                    "measurement": self.measurement_name,
                    "tags": {
                        "type": "translation",
                        "field": "y",
                        "frame_id": self.frame_id
                    },
                    "time": influx_time,
                    "fields": trans_y_fields
                })

            if len(trans_z_fields) > 0:
                self.query.append({
                    "measurement": self.measurement_name,
                    "tags": {
                        "type": "translation",
                        "field": "z",
                        "frame_id": self.frame_id
                    },
                    "time": influx_time,
                    "fields": trans_z_fields
                })

            if len(rot_x_fields) > 0:
                self.query.append({
                    "measurement": self.measurement_name,
                    "tags": {
                        "type": "rotation",
                        "field": "x",
                        "frame_id": self.frame_id
                    },
                    "time": influx_time,
                    "fields": rot_x_fields
                })

            if len(rot_y_fields) > 0:
                self.query.append({
                    "measurement": self.measurement_name,
                    "tags": {
                        "type": "rotation",
                        "field": "y",
                        "frame_id": self.frame_id
                    },
                    "time": influx_time,
                    "fields": rot_y_fields
                })

            if len(rot_z_fields) > 0:
                self.query.append({
                    "measurement": self.measurement_name,
                    "tags": {
                        "type": "rotation",
                        "field": "z",
                        "frame_id": self.frame_id
                    },
                    "time": influx_time,
                    "fields": rot_z_fields
                })

            if len(rot_w_fields) > 0:
                self.query.append({
                    "measurement": self.measurement_name,
                    "tags": {
                        "type": "rotation",
                        "field": "w",
                        "frame_id": self.frame_id
                    },
                    "time": influx_time,
                    "fields": rot_w_fields
                })

            if len(rot_theta_fields) > 0:
                self.query.append({
                    "measurement": self.measurement_name,
                    "tags": {
                        "type": "rotation",
                        "field": "theta",
                        "frame_id": self.frame_id
                    },
                    "time": influx_time,
                    "fields": rot_theta_fields
                })

    def _timer_cb(self, event):
        start_time = time.time() * 1000
        with self.lock:
            if len(self.query) == 0:
                return
            query = copy.deepcopy(self.query)
            self.query = []
        end_time = time.time() * 1000
        rospy.logdebug("copy time: {}ms".format(end_time - start_time))
        rospy.logdebug("data length: {}".format(len(query)))
        try:
            self.client.write_points(query, time_precision='ms')
        except influxdb.exceptions.InfluxDBServerError as e:
            rospy.logerr("InfluxDB error: {}".format(e))
        end_time = time.time() * 1000
        rospy.logdebug("timer cb time: {}ms".format(end_time - start_time))
        if ((end_time - start_time) > (self.duration * 1000)):
            rospy.logerr("timer cb time exceeds: {} > {}".format(
                end_time - start_time, self.duration * 1000))

    def _graph_cb(self, event):
        graph = {}
        try:
            graph = yaml.load(self.get_frames().frame_yaml)
        except Exception as e:
            rospy.logerr(e)

        if len(graph) > 0:
            with self.graph_lock:
                self.graph = graph
