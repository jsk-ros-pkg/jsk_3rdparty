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

import rospy
import rospkg
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


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='')
    parser.add_argument('-eval', '--evaluate')
    parser.add_argument('-o', '--output')
    parser.add_argument('text')
    args = parser.parse_args()

    # check cache_dir
    cache_dir = get_voicevox_cache_dir()
    if not os.path.exists(cache_dir):
        os.makedirs(cache_dir)
    # get speaker_id_to_name data from cache or Client()
    speaker_id_to_name = {}
    if rospy.has_param('voicevox/speakers') and type(rospy.get_param('voicevox/speakers')) is str:
        print('[Text2Wave][{}] Loading speaker id from rosparam'.format(time.time()))
        speaker_id_to_name = json.loads(rospy.get_param('voicevox/speakers'))
    else:
        print('[Text2Wave][{}] Loading speaker id from voicevox server'.format(time.time()))
        speaker_id_to_name = asyncio.run(list_speakers())
        # show speaker ids
        for id, name in sorted(speaker_id_to_name.items(), key=lambda x: int(x[0])):
            print('[Text2Wave][{}] {} : {}'.format(time.time(), id, name))
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

