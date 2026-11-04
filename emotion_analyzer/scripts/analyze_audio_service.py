#!/usr/bin/env python3   
import asyncio
from hume import HumeStreamClient
from hume.models.config import ProsodyConfig, BurstConfig
from emotion_analyzer.srv import AnalyzeAudio
from emotion_analyzer.utils.audio_buffer import AudioBuffer
import soundfile as sf
from pydub import AudioSegment
from io import BytesIO
from base64 import b64encode
import os
import pprint
import json
from std_msgs.msg import String
from audio_common_msgs.msg import AudioInfo

try:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
except ImportError:
    # jsk_common < 3.0 has no jsk_ros1_ros2_compat, so this is ROS1 only:
    # forward ros_compat.xxx(node, ...) to rospy.xxx(...)
    import rospy
    class ros_compat(object):
        create_service = staticmethod(lambda node, *args, **kwargs: rospy.Service(*args, **kwargs))
        create_subscription = staticmethod(lambda node, *args, **kwargs: rospy.Subscriber(*args, **kwargs))
        get_param = staticmethod(lambda node, *args, **kwargs: rospy.get_param(*args, **kwargs))
        get_response_class = staticmethod(lambda srv_type: srv_type._response_class)
        init_node = staticmethod(lambda name, **kwargs: rospy.init_node(name, **kwargs))
        spin = staticmethod(lambda node: rospy.spin())
        loginfo = staticmethod(lambda node, *args, **kwargs: rospy.loginfo(*args, **kwargs))
        logwarn = staticmethod(lambda node, *args, **kwargs: rospy.logwarn(*args, **kwargs))
        logerr = staticmethod(lambda node, *args, **kwargs: rospy.logerr(*args, **kwargs))

AnalyzeAudioResponse = ros_compat.get_response_class(AnalyzeAudio)

class AudioServiceNode:
    def __init__(self, node=None):
        self.node = node
        self.api_key = ros_compat.get_param(node, "hume_api_key", None)
        if not self.api_key:
            ros_compat.logerr(node, "API key has not been set")
            exit(1)

        self.client = HumeStreamClient(self.api_key)
        self.config = [BurstConfig(), ProsodyConfig()]
        self.audio_buffer = AudioBuffer(node=node, topic_name="~audio",
                                        window_size=2.0,
                                        auto_start=True)
        self.expected_coding_format = "wave"
        self.audio_info_sub = ros_compat.create_subscription(node, "/audio/audio_info", AudioInfo, self.audio_info_callback)
        ros_compat.create_service(node, "analyze_audio", AnalyzeAudio, self.handle_request)
        ros_compat.loginfo(node, "Audio-to-Emotion Analysis Service ready.")

    def audio_info_callback(self, msg):
        #check the audio format
        if msg.coding_format != self.expected_coding_format:
            ros_compat.logwarn(self.node, f"Coding_format mismatch: expected {self.expected_coding_format}, got {msg.coding_format}")

    def handle_request(self, req, res=None):
        ros_compat.loginfo(self.node, "Received request for analysis")
        try:
            result = asyncio.run(self.analyze_audio(req.audio_file))
        except Exception as e:
            # an exception here kills the whole rclpy node
            ros_compat.logerr(self.node, "Failed to analyze audio: {}".format(e))
            result = {"error": str(e)}
        ros_compat.loginfo(self.node, "Finished analysis")
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
            if samples is None or len(samples) == 0:
                ros_compat.logwarn(self.node, "Audio data cannot be found. Check the audio topic name.")
                return {"error": "No audio data received."}            
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
        # check the length of the audio
        duration_ms = len(segment)
        if duration_ms > 5000:
            raise Exception(f"Audio is too long: audio length = {duration_ms}ms")

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
                if 'prosody' in result and 'predictions' in result['prosody']:
                    result_prosody = result['prosody']['predictions'][0]['emotions']
                if 'burst' in result and 'predictions' in result['burst']:
                    result_burst = result['burst']['predictions'][0]['emotions']
                # if not predictions:
                #     rospy.logwarn("No predictions found in the result.")
                return {"prosody": result_prosody, "burst": result_burst}
            else:
                ros_compat.logerr(self.node, "Error in receiving valid result.")
                return {"prosody": None, "burst": None}
            #emotions = result["prosody"]["predictions"][0]["emotions"]
            #return str(emotions)  
                   
if __name__ == "__main__":
    node = ros_compat.init_node("analyze_audio_service_node")
    AudioServiceNode(node)
    ros_compat.spin(node)
