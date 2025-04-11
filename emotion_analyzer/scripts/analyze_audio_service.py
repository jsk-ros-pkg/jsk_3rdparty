#!/usr/bin/env python3
import rospy
import asyncio
from hume import HumeStreamClient
from hume.models.config import ProsodyConfig, BurstConfig
from emotion_analyzer.srv import AnalyzeAudio, AnalyzeAudioResponse

class AudioServiceNode:
    def __init__(self):
        self.api_key = rospy.get_param("hume_api_key", None)
        if self.api_key is None:
            rospy.logerr("API key has not been set")
            exit(1)

        self.client = HumeStreamClient(self.api_key)
        self.config = [BurstConfig(), ProsodyConfig()]
        rospy.Service("analyze_audio", AnalyzeAudio, self.handle_request)
        rospy.loginfo("Audio-to-Emotion Analysis Service ready.")

    def handle_request(self, req):
        rospy.loginfo("Received request for analysis")  # requestを受け取ったときにログ
        result = asyncio.run(self.analyze_audio(req.audio_file))
        rospy.loginfo("Finished analysis")  # 結果を返したときにログ
        return AnalyzeAudioResponse(result)

    async def analyze_audio(self, audio_file):
        async with self.client.connect(self.config) as socket:
            result = await socket.send_file(audio_file)
            emotions = result["prosody"]["predictions"][0]["emotions"]
            return str(emotions)  # 必要ならJSON文字列に変換してもOK

if __name__ == "__main__":
    rospy.init_node("analyze_audio_service_node")
    AudioServiceNode()
    rospy.spin()
