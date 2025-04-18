#!/usr/bin/env python3
import rospy
import asyncio
from hume import HumeStreamClient
from hume.models.config import LanguageConfig
from emotion_analyzer.srv import AnalyzeText, AnalyzeTextResponse
import pprint
import json

class TextServiceNode:
    def __init__(self):
        self.api_key = rospy.get_param("hume_api_key", None)
        if self.api_key is None:
            rospy.logerr("API key has not been set")
            exit(1)

        self.client = HumeStreamClient(self.api_key)
        self.config = LanguageConfig(granularity="sentence")
        #granularity="word": analyze each word / granularity="turn": analyze whole text
        rospy.Service("analyze_text", AnalyzeText, self.handle_request)
        rospy.loginfo("Text-to-Emotion Analysis Service ready.")

    def handle_request(self, req):
        rospy.loginfo("Received text for analysis")  
        result = asyncio.run(self.analyze_text(req.text))
        rospy.loginfo("Finished analysis")  
        result_json = json.dumps(result)
        return AnalyzeTextResponse(result_json)

    async def analyze_text(self, text):
        async with self.client.connect([self.config]) as socket:
            result = await socket.send_text(text)
            pprint.pprint(result)
            emotions = result["language"]["predictions"][0]["emotions"]
            return {"emotions": emotions}  # return dict

if __name__ == "__main__":
    rospy.init_node("analyze_text_service_node")
    TextServiceNode()
    rospy.spin()
