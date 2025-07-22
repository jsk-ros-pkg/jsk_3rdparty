#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from dynamic_reconfigure.client import Client
import rospy
from speech_recognition_msgs.srv import SpeechRecognition
from ros_speech_recognition.cfg import SpeechRecognitionConfig as Config


class SpeechRecognitionClient(object):
    def __init__(self,
                 srv_name="/speech_recognition",
                 node_name="/speech_recognition",
                 timeout=10):
        self._sr_srv = rospy.ServiceProxy(srv_name, SpeechRecognition)
        self._sr_srv.wait_for_service(timeout=timeout)
        self._cfg = Client(node_name, timeout=timeout)

    @property
    def language(self):
        return self._cfg.config["language"]

    @language.setter
    def language(self, value):
        self._cfg.update_configuration({'language': value})

    @property
    def engine(self):
        return self._cfg.config["engine"]

    @engine.setter
    def engine(self, value):
        self._cfg.update_configuration({'engine': value})

    @property
    def energy_threshold(self):
        return self._cfg.config["energy_threshold"]

    @energy_threshold.setter
    def energy_threshold(self, value):
        if self.dynamic_energy_threshold:
            rospy.logerr("Dynamic energy thresholding is enabled")
        else:
            self._cfg.update_configuration({
                "energy_threshold": value
            })

    @property
    def dynamic_energy_threshold(self):
        return self._cfg.config["dynamic_energy_threshold"]

    @dynamic_energy_threshold.setter
    def dynamic_energy_threshold(self, value):
        self._cfg.update_configuration({
            "dynamic_energy_threshold": value
        })

    def recognize(self, **args):
        return self._sr_srv(**args).result
