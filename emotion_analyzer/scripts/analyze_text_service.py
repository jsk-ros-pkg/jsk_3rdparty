#!/usr/bin/env python3
import asyncio
from hume import HumeStreamClient
from hume.models.config import LanguageConfig
from emotion_analyzer.srv import AnalyzeText
import pprint
import json

try:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
except ImportError:
    # jsk_common < 3.0 has no jsk_ros1_ros2_compat, so this is ROS1 only:
    # forward ros_compat.xxx(node, ...) to rospy.xxx(...)
    import rospy
    class ros_compat(object):
        create_service = staticmethod(lambda node, *args, **kwargs: rospy.Service(*args, **kwargs))
        get_param = staticmethod(lambda node, *args, **kwargs: rospy.get_param(*args, **kwargs))
        get_response_class = staticmethod(lambda srv_type: srv_type._response_class)
        init_node = staticmethod(lambda name, **kwargs: rospy.init_node(name, **kwargs))
        spin = staticmethod(lambda node: rospy.spin())
        loginfo = staticmethod(lambda node, *args, **kwargs: rospy.loginfo(*args, **kwargs))
        logerr = staticmethod(lambda node, *args, **kwargs: rospy.logerr(*args, **kwargs))

AnalyzeTextResponse = ros_compat.get_response_class(AnalyzeText)

class TextServiceNode:
    def __init__(self, node=None):
        self.node = node
        self.api_key = ros_compat.get_param(node, "hume_api_key", None)
        if self.api_key is None:
            ros_compat.logerr(node, "API key has not been set")
            exit(1)

        self.client = HumeStreamClient(self.api_key)
        self.config = LanguageConfig(granularity="sentence")
        #granularity="word": analyze each word / granularity="turn": analyze whole text
        ros_compat.create_service(node, "analyze_text", AnalyzeText, self.handle_request)
        ros_compat.loginfo(node, "Text-to-Emotion Analysis Service ready.")

    def handle_request(self, req, res=None):
        ros_compat.loginfo(self.node, "Received text for analysis")
        try:
            result = asyncio.run(self.analyze_text(req.text))
        except Exception as e:
            # an exception here kills the whole rclpy node
            ros_compat.logerr(self.node, "Failed to analyze text: {}".format(e))
            result = {"error": str(e)}
        ros_compat.loginfo(self.node, "Finished analysis")
        result_json = json.dumps(result)
        return AnalyzeTextResponse(result=result_json)

    async def analyze_text(self, text):
        async with self.client.connect([self.config]) as socket:
            result = await socket.send_text(text)
            pprint.pprint(result)
            emotions = result["language"]["predictions"][0]["emotions"]
            return {"emotions": emotions}  # return dict

if __name__ == "__main__":
    node = ros_compat.init_node("analyze_text_service_node")
    TextServiceNode(node)
    ros_compat.spin(node)
