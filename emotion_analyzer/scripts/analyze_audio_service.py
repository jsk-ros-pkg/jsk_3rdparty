#!/usr/bin/env python3   
import rospy
import asyncio
from hume import HumeStreamClient
from hume.models.config import ProsodyConfig, BurstConfig
from emotion_analyzer.srv import AnalyzeAudio, AnalyzeAudioResponse
from emotion_analyzer.utils.audio_buffer import AudioBuffer
import soundfile as sf
from pydub import AudioSegment
from io import BytesIO
from base64 import b64encode
import os
import pprint
import json
from std_msgs.msg import String

class AudioServiceNode:
    def __init__(self):
        self.api_key = rospy.get_param("hume_api_key", None)
        if not self.api_key:
            rospy.logerr("API key has not been set")
            exit(1)

        self.client = HumeStreamClient(self.api_key)
        self.config = [BurstConfig(), ProsodyConfig()]
        self.audio_buffer = AudioBuffer(topic_name="~audio",
                                        window_size=2.0,
                                        auto_start=True)
        rospy.Service("analyze_audio", AnalyzeAudio, self.handle_request)
        rospy.loginfo("Audio-to-Emotion Analysis Service ready.")

    def handle_request(self, req):
        rospy.loginfo("Received request for analysis")  # requestを受け取ったときにログ
        result = asyncio.run(self.analyze_audio(req.audio_file))
        rospy.loginfo("Finished analysis")  # 結果を返したときにログ
        if isinstance(result, dict):
            result_json = json.dumps(result)
        else:
            result_json = str(result)
        return AnalyzeAudioResponse(result=result_json)

    async def analyze_audio(self, audio_file):
        if audio_file:
            segment = AudioSegment.from_file(audio_file)
            wav_bytes = segment.raw_data
            sample_width = segment.sample_width
            channels = segment.channels
            frame_rate = segment.frame_rate
        else:
            samples = self.audio_buffer.read()
            wav_bytes = samples.tobytes()
            sample_width = self.audio_buffer.bitdepth // 8
            channels = self.audio_buffer.n_channel
            frame_rate = self.audio_buffer.input_sample_rate
            segment = AudioSegment(
                data=wav_bytes,
                sample_width=sample_width,
                channels=channels,
                frame_rate=frame_rate,
            )
        # Humeに送信（必要に応じて制限時間確認）
        duration_ms = len(segment)
        if duration_ms > 5000:
            raise Exception(f"Hume API制限超過: 音声長 = {duration_ms}ms")

        buf = BytesIO()
        segment.export(buf, format="wav")
        wav_bytes = buf.getvalue()
        b64_audio_str = b64encode(wav_bytes).decode("utf-8")

        async with self.client.connect(self.config) as socket:
            result = await socket.send_bytes(b64_audio_str.encode("utf-8"))
            pprint.pprint(result)

            result_prosody = None
            result_burst = None

            if result and isinstance(result, dict):
                # 予測結果を取得
                if 'prosody' in result and 'predictions' in result['prosody']:
                    result_prosody = result['prosody']['predictions'][0]['emotions']
                if 'burst' in result and 'predictions' in result['burst']:
                    result_burst = result['burst']['predictions'][0]['emotions']
            
                # 予測結果がない場合の処理を追加する
                # if not predictions:
                #     rospy.logwarn("No predictions found in the result.")
                return {"prosody": result_prosody, "burst": result_burst}
            else:
                rospy.logerr("Error in receiving valid result.")
                return {"prosody": None, "burst": None}
            #emotions = result["prosody"]["predictions"][0]["emotions"]
            #return str(emotions)  # 必要ならJSON文字列に変換してもOK
                   
if __name__ == "__main__":
    rospy.init_node("analyze_audio_service_node")
    AudioServiceNode()
    rospy.spin()
