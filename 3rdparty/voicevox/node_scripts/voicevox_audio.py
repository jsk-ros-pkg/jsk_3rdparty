#!/usr/bin/env python3
"""Synthesize text and stream raw PCM to an audio_common subscriber (ROS 2)."""

import io
import json
import math
import time
import urllib.error
import urllib.parse
import urllib.request
import wave

import rclpy
from rclpy.node import Node
from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
from rclpy.qos import DurabilityPolicy, QoSProfile
from audio_common_msgs.msg import AudioData, AudioInfo
from std_msgs.msg import String


def decode_wav(data, sample_rate):
    """Remove the WAV header; audio_common's 'wave' stream is raw PCM."""
    with wave.open(io.BytesIO(data), 'rb') as wav:
        if (wav.getnchannels() != 1 or wav.getsampwidth() != 2
                or wav.getframerate() != sample_rate or wav.getcomptype() != 'NONE'):
            raise ValueError('Expected mono S16LE WAV at %d Hz' % sample_rate)
        frames = wav.getnframes()
        pcm = wav.readframes(frames)
        if not frames or len(pcm) != frames * 2:
            raise ValueError('Empty or truncated WAV from VOICEVOX')
        return pcm


class VoicevoxAudio(Node):
    def __init__(self):
        super().__init__('voicevox_audio')
        for name, default in [('host', 'localhost'), ('port', 50021),
                              ('speaker', 2), ('sample_rate', 16000),
                              ('volume_scale', 3.0),
                              ('startup_timeout', 300.0)]:
            self.declare_parameter(name, default)
        self.rate = self.get_parameter('sample_rate').value
        if self.rate <= 0:
            raise ValueError('sample_rate must be positive')
        self.volume_scale = self.get_parameter('volume_scale').value
        if not math.isfinite(self.volume_scale) or self.volume_scale < 0:
            raise ValueError('volume_scale must be finite and non-negative')
        self.url = 'http://%s:%s' % (self.get_parameter('host').value,
                                   self.get_parameter('port').value)
        self.audio = self.create_publisher(AudioData, 'audio', 10)
        self.info = self.create_publisher(
            AudioInfo, 'audio_info', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.metadata = AudioInfo(channels=1, sample_rate=self.rate,
                                  sample_format='S16LE', bitrate=self.rate * 16,
                                  coding_format='wave')
        self.info.publish(self.metadata)
        self.subscription = self.create_subscription(String, 'text', self.speak, 10)
        self.get_logger().info('Ready: text -> VOICEVOX -> audio (mono S16LE, %d Hz)' % self.rate)

    def request(self, path, params, body=None):
        request = urllib.request.Request(
            self.url + path + '?' + urllib.parse.urlencode(params),
            data=b'' if body is None else json.dumps(body).encode(),
            headers={'Content-Type': 'application/json'}, method='POST')
        with urllib.request.urlopen(request, timeout=120) as response:
            return response.read()

    def synthesize(self, text):
        deadline = time.monotonic() + self.get_parameter('startup_timeout').value
        warned = False
        while rclpy.ok():
            try:
                speaker = self.get_parameter('speaker').value
                query = json.loads(self.request('/audio_query', {'text': text, 'speaker': speaker}))
                query.update(outputSamplingRate=self.rate, outputStereo=False,
                             volumeScale=self.volume_scale)
                return decode_wav(self.request('/synthesis', {'speaker': speaker}, query), self.rate)
            except (urllib.error.URLError, TimeoutError) as error:
                if isinstance(error, urllib.error.HTTPError) and error.code < 500:
                    raise
                if time.monotonic() >= deadline:
                    raise
                if not warned:
                    self.get_logger().warning('Waiting for VOICEVOX engine: %s' % error)
                    warned = True
                time.sleep(1)
        return b''

    def speak(self, message):
        if not message.data.strip():
            return
        try:
            pcm = self.synthesize(message.data)
            if not rclpy.ok():
                return
            if self.audio.get_subscription_count() == 0:
                raise RuntimeError('No subscriber on audio; start the audio playback node')
            self.info.publish(self.metadata)
            # Metadata and PCM use separate DDS topics. Allow metadata to arrive first.
            time.sleep(0.2)
            chunk_bytes = max(1, self.rate // 10) * 2
            start = time.monotonic()
            for offset in range(0, len(pcm), chunk_bytes):
                if not rclpy.ok():
                    return
                self.audio.publish(AudioData(data=pcm[offset:offset + chunk_bytes]))
                end = start + min(offset + chunk_bytes, len(pcm)) / (self.rate * 2)
                time.sleep(max(0, end - time.monotonic()))
            self.get_logger().info('Published %.2f s of speech' % (len(pcm) / (self.rate * 2)))
        except Exception as error:
            self.get_logger().error('Speech failed: %s' % error)


def main():
    rclpy.init()
    ros_compat.spin_and_shutdown(VoicevoxAudio())


if __name__ == '__main__':
    main()
