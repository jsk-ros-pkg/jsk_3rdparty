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
        if self.api_key is None:
            rospy.logerr("API key has not been set")
            exit(1)

        self.client = HumeStreamClient(self.api_key)
        self.config = [BurstConfig(), ProsodyConfig()]
        self.audio_topic = rospy.get_param('~audio_topic', '/audio')
        self.audio_buffer = AudioBuffer(topic_name=self.audio_topic,
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

    def record_audio_to_tempfile(self):
        wav_outpath = '/home/leus/tmp/hoge.wav'
        bytes_io = BytesIO()
        with sf.SoundFile(wav_outpath, mode='w',
                          samplerate=self.audio_buffer.input_sample_rate,
                          channels=self.audio_buffer.n_channel,
                          format='wav') as f:
            tmp = self.audio_buffer.read()
            f.write(tmp)
        return wav_outpath

    async def analyze_audio(self, audio_file):
        if audio_file:
            wav_path = audio_file
        else:
            rospy.loginfo("Recording audio from topic: %s", self.audio_topic)
            wav_path = self.record_audio_to_tempfile()

        # Humeに送信（必要に応じて制限時間確認）
        segment = AudioSegment.from_file(wav_path, format="wav")
        duration_ms = len(segment)
        if duration_ms > 5000:
            raise Exception(f"Hume API制限超過: 音声長 = {duration_ms}ms")

        async with self.client.connect(self.config) as socket:
            result = await socket.send_file(wav_path)
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
