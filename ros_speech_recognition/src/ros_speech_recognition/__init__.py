from ros_speech_recognition.compat import ROS_VERSION
if ROS_VERSION == 1:  # client.py is ROS1 only (rospy, dynamic_reconfigure)
    from ros_speech_recognition.client import SpeechRecognitionClient  # NOQA
