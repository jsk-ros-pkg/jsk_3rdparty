#!/usr/bin/env python3
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
    return os.path.join(
        os.environ.get('ROS_HOME', os.path.expanduser('~/.ros')),
        'voicevox')

def get_speakers_cache_path():
    return os.path.join(get_voicevox_cache_dir(), 'speakers.json')

def load_speakers_cache():
    path = get_speakers_cache_path()
    if not os.path.exists(path):
        return None
    with open(path, encoding='utf-8') as f:
        return json.load(f)

def save_speakers_cache(speaker_id_to_name):
    if not os.path.exists(get_voicevox_cache_dir()):
        os.makedirs(get_voicevox_cache_dir())
    with open(get_speakers_cache_path(), 'w', encoding='utf-8') as f:
        json.dump(speaker_id_to_name, f, ensure_ascii=False)

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
    speaker_id_to_name = load_speakers_cache()
    if speaker_id_to_name is not None:
        print('[Text2Wave][{}] Loading speaker id from cache'.format(time.time()))
    else:
        print('[Text2Wave][{}] Loading speaker id from voicevox server'.format(time.time()))
        speaker_id_to_name = asyncio.run(list_speakers())
        # show speaker ids
        for id, name in sorted(speaker_id_to_name.items(), key=lambda x: int(x[0])):
            print('[Text2Wave][{}] {} : {}'.format(time.time(), id, name))
        save_speakers_cache(speaker_id_to_name)

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
