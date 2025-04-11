#!/usr/bin/env python3
import rospy
import asyncio
from hume import HumeStreamClient
from hume.models.config import LanguageConfig
from emotion_analyzer.srv import AnalyzeText, AnalyzeTextResponse

class HumeServiceNode:
    def __init__(self):
        self.api_key = rospy.get_param("hume_api_key", None)
        if self.api_key is None:
            rospy.logerr("API key has not been set")
            exit(1)

        self.client = HumeStreamClient(self.api_key)
        self.config = LanguageConfig()
        rospy.Service("analyze_text", AnalyzeText, self.handle_request)
        rospy.loginfo("Hume Service ready.")

    def handle_request(self, req):
        rospy.loginfo("Received text for analysis")  # テキストを受け取ったときにログ
        result = asyncio.run(self.analyze_text(req.text))
        rospy.loginfo("Finished analysis")  # 結果を返したときにログ
        return AnalyzeTextResponse(result)

    async def analyze_text(self, text):
        async with self.client.connect([self.config]) as socket:
            result = await socket.send_text(text)
            emotions = result["language"]["predictions"][0]["emotions"]
            return str(emotions)  # 必要ならJSON文字列に変換してもOK

if __name__ == "__main__":
    rospy.init_node("analyze_text_service_node")
    HumeServiceNode()
    rospy.spin()
