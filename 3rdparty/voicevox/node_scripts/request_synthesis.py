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
# wait for the engine (e.g. still starting up) for up to this many seconds
VOICEVOX_STARTUP_TIMEOUT = float(os.environ.get(
    'VOICEVOX_STARTUP_TIMEOUT', 300))
VOICEVOX_RETRY_INTERVAL = float(os.environ.get(
    'VOICEVOX_RETRY_INTERVAL', 1))
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


def run_with_retry(coroutine_function, *args):
    """asyncio.run(coroutine_function(*args)), retrying while the engine can
    not be reached (e.g. it is still starting up) for up to
    VOICEVOX_STARTUP_TIMEOUT seconds. Errors from the engine itself (an
    invalid request) are raised at once."""
    import httpx
    deadline = time.time() + VOICEVOX_STARTUP_TIMEOUT
    retry = 0
    while True:
        try:
            return asyncio.run(coroutine_function(*args))
        except httpx.TransportError as e:
            if time.time() >= deadline:
                raise
            if retry % 10 == 0:
                print('[Text2Wave][{}] Waiting for VOICEVOX engine at {}:{}: {}'.format(
                    time.time(), VOICEVOX_TEXTTOSPEECH_URL, VOICEVOX_TEXTTOSPEECH_PORT, e))
            retry += 1
            time.sleep(VOICEVOX_RETRY_INTERVAL)


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
        speaker_id_to_name = run_with_retry(list_speakers)
        # show speaker ids
        for id, name in sorted(speaker_id_to_name.items(), key=lambda x: int(x[0])):
            print('[Text2Wave][{}] {} : {}'.format(time.time(), id, name))
        with open(speakers_cache, 'w', encoding='utf-8') as f:
            json.dump(speaker_id_to_name, f, ensure_ascii=False)
    return speaker_id_to_name


def get_speaker_id(speaker_name):
    """speaker_name: a speaker id, or (the beginning of) a speaker name"""
    speaker_id = VOICEVOX_DEFAULT_SPEAKER_ID
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
    run_with_retry(request_synthesis,
                   speech_text, output_path, speaker_id,
                   sample_rate, volume_scale)

    if cache_enabled :
        shutil.copy(output_path, cache_filename)


def run_audio_node():
    """ROS node that speaks the text it subscribes on text (std_msgs/String)
    or robotsound (sound_play's SoundRequest, e.g. from SoundClient.say())
    by publishing the synthesized speech as raw PCM (mono S16LE) on audio
    (audio_common_msgs/AudioData), for e.g. audio_common's audio_play with
    format:=wave, instead of writing a wav file for sound_play"""
    import tempfile
    import threading
    import wave
    from audio_common_msgs.msg import AudioData, AudioInfo
    from std_msgs.msg import String
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
    if ros_compat.ROS_VERSION == 1:
        import actionlib
        import rospy
        from sound_play.msg import (SoundRequest, SoundRequestAction,
                                    SoundRequestFeedback, SoundRequestResult)
    else:
        from rclpy.action import ActionServer, CancelResponse, GoalResponse
        from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
        from sound_play_msgs.msg import SoundRequest
        from sound_play_msgs.action import SoundRequest as SoundRequestAction
        from builtin_interfaces.msg import Duration
        SoundRequestFeedback = SoundRequestAction.Feedback
        SoundRequestResult = SoundRequestAction.Result

    node = ros_compat.init_node('voicevox_audio', multi_threaded=True)
    speech_lock = threading.Lock()
    sample_rate = ros_compat.get_param(node, '~sample_rate', 16000)
    volume_scale = ros_compat.get_param(node, '~volume_scale', 1.0)
    pub_audio = ros_compat.create_publisher(node, 'audio', AudioData, queue_size=10)
    pub_audio_info = ros_compat.create_publisher(
        node, 'audio_info', AudioInfo, queue_size=1, latch=True)
    audio_info = AudioInfo(
        channels=1,
        sample_rate=sample_rate,
        sample_format='S16LE',
        bitrate=sample_rate * 16,
        coding_format='wave')
    pub_audio_info.publish(audio_info)

    def speak(text, speaker='', canceled=lambda: False, feedback=None, volume=1.0):
        # Serialize topic and action requests so their PCM chunks never interleave.
        if not speech_lock.acquire(False):
            raise RuntimeError('Another speech request is already being processed')
        try:
            if canceled() or ros_compat.is_shutdown(node):
                return False
            speaker_id = get_speaker_id(
                speaker or ros_compat.get_param(node, '~speaker', VOICEVOX_DEFAULT_SPEAKER_ID))
            with tempfile.NamedTemporaryFile(suffix='.wav') as f:
                synthesize(text, speaker_id, f.name, sample_rate, volume_scale * volume)
                with wave.open(f.name, 'rb') as w:
                    pcm = w.readframes(w.getnframes())
            # Publish in 0.1 sec chunks at playback speed.
            chunk_bytes = max(1, sample_rate // 10) * 2
            start = time.monotonic()
            for offset in range(0, len(pcm), chunk_bytes):
                if canceled() or ros_compat.is_shutdown(node):
                    return False
                chunk = pcm[offset:offset + chunk_bytes]
                pub_audio.publish(AudioData(data=chunk))
                elapsed = (offset + len(chunk)) / (sample_rate * 2.0)
                if feedback is not None:
                    feedback(elapsed)
                time.sleep(max(0, start + elapsed - time.monotonic()))
            if canceled() or ros_compat.is_shutdown(node):
                return False
            ros_compat.loginfo(node, 'Published {:.2f} sec of speech'.format(len(pcm) / (sample_rate * 2.0)))
            return True
        finally:
            speech_lock.release()

    def topic_speak(text, speaker=''):
        try:
            speak(text, speaker)
        except Exception as e:
            ros_compat.logerr(node, 'Failed to synthesize {}: {}'.format(text, e))

    def supported(goal):
        req = goal.sound_request
        return req.sound == SoundRequest.SAY and req.command == SoundRequest.PLAY_ONCE

    def execute(goal, canceled, publish_feedback):
        result = SoundRequestResult()

        def feedback(elapsed):
            msg = SoundRequestFeedback()
            msg.playing = True
            if ros_compat.ROS_VERSION == 1:
                msg.stamp = rospy.Time.from_sec(elapsed)
            else:
                ns = int(elapsed * 1e9)
                msg.stamp = Duration(sec=ns // 1000000000, nanosec=ns % 1000000000)
            result.stamp = msg.stamp
            publish_feedback(msg)

        try:
            if not supported(goal):
                raise ValueError('Only SAY with PLAY_ONCE is supported')
            req = goal.sound_request
            done = speak(req.arg, req.arg2, canceled, feedback, req.volume)
            status = 'succeeded' if done else 'preempted'
        except Exception as e:
            ros_compat.logerr(node, 'Speech action failed: {}'.format(e))
            status = 'preempted' if canceled() else 'aborted'
        result.playing = False
        return status, result

    if ros_compat.ROS_VERSION == 1:
        def execute_ros1(goal):
            status, result = execute(goal, server.is_preempt_requested, server.publish_feedback)
            getattr(server, 'set_' + status)(result)

        server = actionlib.SimpleActionServer(
            'robotsound', SoundRequestAction, execute_cb=execute_ros1, auto_start=False)
        server.start()
    else:
        # Accept one action at a time; cancellation must run during execution.
        action_lock = threading.Lock()

        def accept_goal(goal):
            if supported(goal) and action_lock.acquire(False):
                return GoalResponse.ACCEPT
            return GoalResponse.REJECT

        def execute_ros2(handle):
            try:
                status, result = execute(
                    handle.request, lambda: handle.is_cancel_requested, handle.publish_feedback)
                if handle.is_cancel_requested:
                    handle.canceled()
                elif status == 'succeeded':
                    handle.succeed()
                else:
                    handle.abort()
                return result
            finally:
                action_lock.release()

        server = ActionServer(
            node, SoundRequestAction, 'robotsound', execute_ros2,
            goal_callback=accept_goal,
            cancel_callback=lambda handle: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())

    def sound_request_callback(msg):
        # arg2 is the voice of SoundClient.say(text, voice=...)
        if msg.sound == SoundRequest.SAY and msg.command == SoundRequest.PLAY_ONCE:
            topic_speak(msg.arg, msg.arg2)
    topic_group = MutuallyExclusiveCallbackGroup() if ros_compat.ROS_VERSION == 2 else None
    ros_compat.create_subscription(
        node, 'text', String, lambda msg: topic_speak(msg.data), queue_size=10,
        callback_group=topic_group)
    ros_compat.create_subscription(
        node, 'robotsound', SoundRequest, sound_request_callback, queue_size=10,
        callback_group=topic_group)
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
        'that publishes the speech of the text it subscribes on text '
        '(std_msgs/String) or robotsound (SoundRequest) as raw PCM on audio '
        '(audio_common_msgs/AudioData)')
    parser.add_argument('-eval', '--evaluate')
    parser.add_argument('-o', '--output')
    parser.add_argument('text')
    args = parser.parse_args()

    with open(args.text, 'rb') as f:
        speech_text = convert_to_str(f.readline())

    # get speaker_id
    speaker_id = get_speaker_id(args.evaluate)
    synthesize(speech_text, speaker_id, args.output)
