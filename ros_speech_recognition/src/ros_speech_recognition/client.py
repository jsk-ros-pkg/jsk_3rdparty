#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from speech_recognition_msgs.srv import SpeechRecognition


class SpeechRecognitionClient:
    def __init__(self,
                 srv_name='/speech_recognition',
                 node_name='/speech_recognition',
                 node=None,
                 timeout=10):
        self._owns_node = node is None
        self._owns_context = self._owns_node and not rclpy.ok()
        if self._owns_node:
            if self._owns_context:
                rclpy.init()
            node = Node('speech_recognition_client')
        self.node = node
        self.timeout = float(timeout)
        self._service = node.create_client(SpeechRecognition, srv_name)
        if not self._service.wait_for_service(timeout_sec=self.timeout):
            raise TimeoutError(
                'Speech recognition service is not available: {}'.format(
                    srv_name))
        self._parameters = AsyncParameterClient(node, node_name)
        if not self._parameters.wait_for_services(
                timeout_sec=self.timeout):
            raise TimeoutError(
                'Parameter services are not available: {}'.format(node_name))

    def close(self):
        if self._owns_node:
            self.node.destroy_node()
        if self._owns_context:
            rclpy.try_shutdown()

    def _wait(self, future):
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=self.timeout)
        if not future.done():
            raise TimeoutError('ROS 2 request timed out')
        if future.exception() is not None:
            raise future.exception()
        return future.result()

    def _get_parameter(self, name):
        response = self._wait(self._parameters.get_parameters([name]))
        return parameter_value_to_python(response.values[0])

    def _set_parameter(self, name, value):
        response = self._wait(self._parameters.set_parameters([
            Parameter(name, value=value),
        ]))
        result = response.results[0]
        if not result.successful:
            raise ValueError(result.reason)

    @property
    def language(self):
        return self._get_parameter('language')

    @language.setter
    def language(self, value):
        self._set_parameter('language', value)

    @property
    def engine(self):
        return self._get_parameter('engine')

    @engine.setter
    def engine(self, value):
        self._set_parameter('engine', value)

    @property
    def energy_threshold(self):
        return self._get_parameter('energy_threshold')

    @energy_threshold.setter
    def energy_threshold(self, value):
        if self.dynamic_energy_threshold:
            raise ValueError('Dynamic energy thresholding is enabled')
        else:
            self._set_parameter('energy_threshold', float(value))

    @property
    def dynamic_energy_threshold(self):
        return self._get_parameter('dynamic_energy_threshold')

    @dynamic_energy_threshold.setter
    def dynamic_energy_threshold(self, value):
        self._set_parameter('dynamic_energy_threshold', bool(value))

    def recognize(self, **args):
        request = SpeechRecognition.Request()
        for name, value in args.items():
            if not hasattr(request, name):
                raise AttributeError(
                    'SpeechRecognition request has no field {}'.format(name))
            setattr(request, name, value)
        return self._wait(self._service.call_async(request)).result
