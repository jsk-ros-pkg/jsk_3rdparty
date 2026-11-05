# -*- coding: utf-8 -*-
"""ROS2 only, installed as ros_speech_recognition.cfg.SpeechRecognitionConfig:
dynamic_reconfigure has no ROS2 port, and its parameters are ROS2
parameters (`ros2 param set`) declared with these `defaults` by
ros_compat.create_reconfigure_server(). Keep them the same as
../SpeechRecognition.cfg."""

SpeechRecognition_Google = 'Google'
SpeechRecognition_GoogleCloud = 'GoogleCloud'
SpeechRecognition_GoogleCloudStream = 'GoogleCloudStream'
SpeechRecognition_Sphinx = 'Sphinx'
SpeechRecognition_Wit = 'Wit'
SpeechRecognition_Bing = 'Bing'
SpeechRecognition_Houndify = 'Houndify'
SpeechRecognition_IBM = 'IBM'
SpeechRecognition_Vosk = 'Vosk'

defaults = {
    'language': 'en-US',
    'engine': SpeechRecognition_Google,
    'energy_threshold': 300.0,
    'dynamic_energy_threshold': True,
    'dynamic_energy_adjustment_damping': 0.15,
    'dynamic_energy_ratio': 1.5,
    'pause_threshold': 0.8,
    'operation_timeout': 0.0,
    'listen_timeout': 0.0,
    'phrase_time_limit': 10.0,
    'phrase_threshold': 0.3,
    'non_speaking_duration': 0.5,
}
