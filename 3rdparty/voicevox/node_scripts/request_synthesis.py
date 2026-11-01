#!/usr/bin/env python
# -*- coding:utf-8 -*-

from voicevox import Client
import asyncio

import argparse
import hashlib
import os
import shutil
import sys
import time

import json

VOICEVOX_DEFAULT_SPEAKER_ID = os.environ.get(
    'VOICEVOX_DEFAULT_SPEAKER_ID', 2)
VOICEVOX_TEXTTOSPEECH_URL = os.environ.get(
    'VOICEVOX_TEXTTOSPEECH_URL', 'localhost')
VOICEVOX_TEXTTOSPEECH_PORT = os.environ.get(
    'VOICEVOX_TEXTTOSPEECH_PORT', '50021')
cache_enabled = os.environ.get(
    'ROS_VOICEVOX_TEXTTOSPEECH_CACHE_ENABLED', True)
cache_enabled = cache_enabled is True \
    or cache_enabled == 'true'  # for launch env tag.

def convert_to_str(x):
    if isinstance(x, str):
        pass
    elif isinstance(x, bytes):
        x = x.decode('utf-8')
    else:
        raise ValueError(
            'Invalid input x type: {}'
            .format(type(x)))
    return x

def get_voicevox_cache_dir():
    # same as rospkg.get_ros_home(), which ROS2 does not have
    return os.environ.get('ROS_HOME', os.path.join(os.path.expanduser('~'), '.ros')) + '/voicevox/'

async def request_synthesis(
        sentence, output_path, speaker_id='1',
        sample_rate=None, volume_scale=None):
    async with Client(base_url='http://'+VOICEVOX_TEXTTOSPEECH_URL+':'+VOICEVOX_TEXTTOSPEECH_PORT) as client:
        audio_query = await client.create_audio_query(sentence, speaker=speaker_id)
        if sample_rate is not None:
            audio_query.output_sampling_rate = sample_rate
            audio_query.output_stereo = False
        if volume_scale is not None:
            audio_query.volume_scale = volume_scale
        with open(output_path, "wb") as f:
            f.write(await audio_query.synthesis(speaker=speaker_id))

async def list_speakers():
    speaker_id_to_name = {}
    async with Client(base_url='http://'+VOICEVOX_TEXTTOSPEECH_URL+':'+VOICEVOX_TEXTTOSPEECH_PORT) as client:
        for speaker in await client.fetch_speakers():
            for styles in speaker.styles:
                speaker_id_to_name[styles.id] = speaker.name + '-' + styles.name
    return speaker_id_to_name


def get_speaker_id_to_name():
    # check cache_dir
    cache_dir = get_voicevox_cache_dir()
    if not os.path.exists(cache_dir):
        os.makedirs(cache_dir)
    # get speaker_id_to_name data from cache or Client()
    # The cache is a file, as this command runs for each utterance and ROS2
    # has no global parameter server (ROS1 used rosparam voicevox/speakers)
    speaker_id_to_name = {}
    speakers_cache = os.path.join(cache_dir, 'speakers.json')
    if os.path.exists(speakers_cache):
        print('[Text2Wave][{}] Loading speaker id from {}'.format(time.time(), speakers_cache))
        with open(speakers_cache, encoding='utf-8') as f:
            speaker_id_to_name = json.load(f)
    else:
        print('[Text2Wave][{}] Loading speaker id from voicevox server'.format(time.time()))
        speaker_id_to_name = asyncio.run(list_speakers())
        # show speaker ids
        for id, name in sorted(speaker_id_to_name.items(), key=lambda x: int(x[0])):
            print('[Text2Wave][{}] {} : {}'.format(time.time(), id, name))
        with open(speakers_cache, 'w', encoding='utf-8') as f:
            json.dump(speaker_id_to_name, f, ensure_ascii=False)
    return speaker_id_to_name


def get_speaker_id(speaker_name):
    """speaker_name: a speaker id, or (the beginning of) a speaker name"""
    speaker_id = VOICEVOX_DEFAULT_SPEAKER_ID
    print(["speaker_id = ",speaker_id])
    speaker_name = str(speaker_name).lstrip('(').rstrip(')')
    if type(speaker_name) is int or speaker_name.isdigit():
        speaker_id = speaker_name
    else:
        speaker_name = list(filter(
            lambda x: x[1].startswith(speaker_name),
            get_speaker_id_to_name().items()))
        if speaker_name:
            speaker_id = speaker_name[0][0]
    return speaker_id


def synthesize(speech_text, speaker_id, output_path,
               sample_rate=None, volume_scale=None):
    """Write the speech of speech_text to the wav file output_path"""
    if cache_enabled:
        if not os.path.exists(get_voicevox_cache_dir()):
            os.makedirs(get_voicevox_cache_dir())
        cache_key = [hashlib.md5(speech_text.encode('utf-8')).hexdigest(), str(speaker_id)]
        if sample_rate is not None or volume_scale is not None:
            cache_key += [str(sample_rate), str(volume_scale)]
        cache_filename = os.path.join(get_voicevox_cache_dir(), '--'.join(cache_key) + '.wav')
        if os.path.exists(cache_filename):
            print('[Text2Wave][{}] Using cached file ({}) for {}'.format(time.time(),cache_filename, speech_text))
            shutil.copy(cache_filename, output_path)
            return

    # cehck cache
    print('[Text2Wave][{}] speak {} with ({})'.format(time.time(),speech_text, speaker_id))
    asyncio.run(request_synthesis(speech_text,
                                  output_path,
                                  speaker_id,
                                  sample_rate, volume_scale))

    if cache_enabled :
        shutil.copy(output_path, cache_filename)


def run_audio_node():
    """ROS node that speaks the text it subscribes on text (std_msgs/String)
    by publishing the synthesized speech as raw PCM (mono S16LE) on audio
    (audio_common_msgs/AudioData), for e.g. audio_common's audio_play with
    format:=wave, instead of writing a wav file for sound_play"""
    import tempfile
    import wave
    from audio_common_msgs.msg import AudioData
    from std_msgs.msg import String
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat

    node = ros_compat.init_node('voicevox_audio')
    sample_rate = ros_compat.get_param(node, '~sample_rate', 16000)
    volume_scale = ros_compat.get_param(node, '~volume_scale', 1.0)
    pub_audio = ros_compat.create_publisher(node, 'audio', AudioData, queue_size=10)

    def callback(msg):
        # ~speaker is read for each message, so that it can be changed at runtime
        speaker_id = get_speaker_id(
            ros_compat.get_param(node, '~speaker', VOICEVOX_DEFAULT_SPEAKER_ID))
        with tempfile.NamedTemporaryFile(suffix='.wav') as f:
            synthesize(msg.data, speaker_id, f.name, sample_rate, volume_scale)
            with wave.open(f.name, 'rb') as w:
                pcm = w.readframes(w.getnframes())
        # publish in 0.1 sec chunks at the playback speed
        chunk_bytes = sample_rate // 10 * 2
        start = time.time()
        for offset in range(0, len(pcm), chunk_bytes):
            if ros_compat.is_shutdown(node):
                return
            pub_audio.publish(AudioData(data=pcm[offset:offset + chunk_bytes]))
            time.sleep(max(0, start + (offset + chunk_bytes) / (sample_rate * 2.0) - time.time()))
        ros_compat.loginfo(node, 'Published {:.2f} sec of speech'.format(len(pcm) / (sample_rate * 2.0)))
    ros_compat.create_subscription(node, 'text', String, callback)
    ros_compat.spin(node)


if __name__ == '__main__':
    # --output-type is parsed first, as ROS arguments (e.g. remappings)
    # follow it with --output-type audio
    output_type_parser = argparse.ArgumentParser(add_help=False)
    output_type_parser.add_argument('--output-type', choices=['wav', 'audio'], default='wav')
    if output_type_parser.parse_known_args()[0].output_type == 'audio':
        run_audio_node()
        sys.exit(0)

    parser = argparse.ArgumentParser(description='')
    parser.add_argument(
        '--output-type', choices=['wav', 'audio'], default='wav',
        help='wav: write the speech of TEXT (file) to the wav file OUTPUT, '
        'as text2wave for sound_play (default). audio: run as a ROS node '
        'that publishes the speech of the text (std_msgs/String) it '
        'subscribes on text as raw PCM on audio (audio_common_msgs/AudioData)')
    parser.add_argument('-eval', '--evaluate')
    parser.add_argument('-o', '--output')
    parser.add_argument('text')
    args = parser.parse_args()

    with open(args.text, 'rb') as f:
        speech_text = convert_to_str(f.readline())

    # get speaker_id
    speaker_id = get_speaker_id(args.evaluate)
    synthesize(speech_text, speaker_id, args.output)
