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
ROS2 = os.environ.get('ROS_VERSION') == '2'
if not ROS2:
    import rospy
    import rospkg

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
    if ROS2:  # same as rospkg.get_ros_home()
        return os.environ.get('ROS_HOME', os.path.expanduser('~/.ros')) + '/voicevox/'
    return rospkg.get_ros_home() + '/voicevox/'

async def request_synthesis(
        sentence, output_path, speaker_id='1'):
    async with Client(base_url='http://'+VOICEVOX_TEXTTOSPEECH_URL+':'+VOICEVOX_TEXTTOSPEECH_PORT) as client:
        audio_query = await client.create_audio_query(sentence, speaker=speaker_id)
        with open(output_path, "wb") as f:
            f.write(await audio_query.synthesis(speaker=speaker_id))

async def list_speakers():
    speaker_id_to_name = {}
    async with Client(base_url='http://'+VOICEVOX_TEXTTOSPEECH_URL+':'+VOICEVOX_TEXTTOSPEECH_PORT) as client:
        for speaker in await client.fetch_speakers():
            for styles in speaker.styles:
                speaker_id_to_name[styles.id] = speaker.name + '-' + styles.name
    return speaker_id_to_name


async def ros2_synthesize(args):
    """Retry unavailable engines, but fail invalid requests without hiding them."""
    import httpx
    import io
    import tempfile
    import wave

    text = open(args.text, encoding='utf-8').read()
    speaker = (args.evaluate or '').strip('()') or str(VOICEVOX_DEFAULT_SPEAKER_ID)
    if speaker == 'voice_kal_diphone':
        speaker = str(VOICEVOX_DEFAULT_SPEAKER_ID)
    cache_dir = get_voicevox_cache_dir()
    os.makedirs(cache_dir, exist_ok=True)

    def valid_wav(data):
        try:
            with wave.open(io.BytesIO(data)) as wav:
                return wav.getnframes() > 0 and len(wav.readframes(wav.getnframes())) > 0
        except (wave.Error, EOFError):
            return False

    def atomic_write(path, data):
        fd, temporary = tempfile.mkstemp(dir=os.path.dirname(os.path.abspath(path)))
        try:
            with os.fdopen(fd, 'wb') as output:
                output.write(data)
            os.replace(temporary, path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    timeout = float(os.environ.get('VOICEVOX_REQUEST_TIMEOUT', '120'))
    retry_interval = float(os.environ.get('VOICEVOX_RETRY_INTERVAL', '1'))
    base_url = 'http://' + VOICEVOX_TEXTTOSPEECH_URL + ':' + VOICEVOX_TEXTTOSPEECH_PORT
    async with httpx.AsyncClient(base_url=base_url, timeout=httpx.Timeout(timeout, connect=2)) as client:
        async def request(method, path, **kwargs):
            response = await client.request(method, path, **kwargs)
            response.raise_for_status()
            return response

        attempts = 0
        while True:
            try:
                if not speaker.isdigit():
                    response = await request('GET', '/speakers')
                    matches = [str(style['id']) for item in response.json()
                               for style in item['styles']
                               if (item['name'] + '-' + style['name']).startswith(speaker)]
                    if not matches:
                        raise ValueError('Unknown speaker: ' + speaker)
                    speaker = matches[0]
                cache_path = os.path.join(cache_dir, hashlib.md5(text.encode()).hexdigest() + '--' + speaker + '.wav')
                if cache_enabled and os.path.exists(cache_path):
                    data = open(cache_path, 'rb').read()
                    if valid_wav(data):
                        atomic_write(args.output, data)
                        print('[VOICEVOX] Using cached audio', flush=True)
                        return
                response = await request('POST', '/audio_query', params={'text': text, 'speaker': speaker})
                response = await request('POST', '/synthesis', params={'speaker': speaker}, json=response.json())
                data = response.content
                if not valid_wav(data):
                    raise ValueError('VOICEVOX returned invalid or empty WAV audio')
                atomic_write(args.output, data)
                if cache_enabled:
                    atomic_write(cache_path, data)
                print('[VOICEVOX] Synthesis complete', flush=True)
                return
            except (httpx.TransportError, httpx.HTTPStatusError) as error:
                if isinstance(error, httpx.HTTPStatusError) and error.response.status_code < 500:
                    raise
                attempts += 1
                if attempts == 1 or attempts % 10 == 0:
                    print('[VOICEVOX] Waiting/retrying engine at %s: %s' % (base_url, error), flush=True)
                await asyncio.sleep(retry_interval)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='')
    parser.add_argument('-eval', '--evaluate')
    parser.add_argument('-o', '--output')
    parser.add_argument('text')
    args = parser.parse_args()

    if ROS2:
        try:
            asyncio.run(asyncio.wait_for(
                ros2_synthesize(args),
                timeout=float(os.environ.get('VOICEVOX_STARTUP_TIMEOUT', '300'))))
        except Exception as error:
            print('[VOICEVOX] Synthesis failed: %s: %s' % (type(error).__name__, error), file=sys.stderr, flush=True)
            sys.exit(1)
        sys.exit(0)

    # check cache_dir
    cache_dir = get_voicevox_cache_dir()
    if not os.path.exists(cache_dir):
        os.makedirs(cache_dir)
    # get speaker_id_to_name data from cache or Client()
    speaker_id_to_name = {}
    speakers_cache = cache_dir + 'speakers.json'  # ROS 2 has no global parameter server
    if ROS2 and os.path.exists(speakers_cache):
        print('[Text2Wave][{}] Loading speaker id from cache'.format(time.time()))
        speaker_id_to_name = json.load(open(speakers_cache, encoding='utf-8'))
    elif not ROS2 and rospy.has_param('voicevox/speakers') and type(rospy.get_param('voicevox/speakers')) is str:
        print('[Text2Wave][{}] Loading speaker id from rosparam'.format(time.time()))
        speaker_id_to_name = json.loads(rospy.get_param('voicevox/speakers'))
    else:
        print('[Text2Wave][{}] Loading speaker id from voicevox server'.format(time.time()))
        speaker_id_to_name = asyncio.run(list_speakers())
        # show speaker ids
        for id, name in sorted(speaker_id_to_name.items(), key=lambda x: int(x[0])):
            print('[Text2Wave][{}] {} : {}'.format(time.time(), id, name))
        if ROS2:
            json.dump(speaker_id_to_name, open(speakers_cache, 'w', encoding='utf-8'), ensure_ascii=False)
        else:
            rospy.set_param('voicevox/speakers', json.dumps(speaker_id_to_name, ensure_ascii=False))

    with open(args.text, 'rb') as f:
        speech_text = convert_to_str(f.readline())

    # get speaker_id
    speaker_id = VOICEVOX_DEFAULT_SPEAKER_ID
    print(["speaker_id = ",speaker_id])
    speaker_name = args.evaluate.lstrip('(').rstrip(')')
    if type(speaker_name) is int or speaker_name.isdigit():
        speaker_id = speaker_name
    else:
        speaker_name = list(filter(
            lambda x: x[1].startswith(speaker_name),
            speaker_id_to_name.items()))
        if speaker_name:
            speaker_id = speaker_name[0][0]

    if cache_enabled:
        cache_filename = os.path.join(get_voicevox_cache_dir(), '--'.join([hashlib.md5(speech_text.encode('utf-8')).hexdigest(), str(speaker_id)]) + '.wav')
        if os.path.exists(cache_filename):
            print('[Text2Wave][{}] Using cached file ({}) for {}'.format(time.time(),cache_filename, speech_text))
            shutil.copy(cache_filename, args.output)
            sys.exit(0)

    # cehck cache
    print('[Text2Wave][{}] speak {} with {}({})'.format(time.time(),speech_text, speaker_name, speaker_id))
    asyncio.run(request_synthesis(speech_text,
                                  args.output,
                                  speaker_id))

    if cache_enabled :
        shutil.copy(args.output, cache_filename)

