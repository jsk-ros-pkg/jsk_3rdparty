#!/usr/bin/env python
# -*- coding:utf-8 -*-

from voicevox import Client
import asyncio

import argparse
import os
import shutil
import sys

#import requests

# from voicevox.filecheck_utils import checksum_md5
# from voicevox.filecheck_utils import get_cache_dir


speaker_id_to_name = {
    '0': '四国めたん-あまあま',
    '1': 'ずんだもん-あまあま',
    '2': '四国めたん-ノーマル',
    '3': 'ずんだもん-ノーマル',
    '4': '四国めたん-セクシー',
    '5': 'ずんだもん-セクシー',
    '6': '四国めたん-ツンツン',
    '7': 'ずんだもん-ツンツン',
    '8': '春日部つむぎ-ノーマル',
    '9': '波音リツ-ノーマル',
    '10': '雨晴はう-ノーマル',
    '11': '玄野武宏-ノーマル',
    '12': '白上虎太郎-ノーマル',
    '13': '青山龍星-ノーマル',
    '14': '冥鳴ひまり-ノーマル',
    '15': '九州そら-あまあま',
    '16': '九州そら-ノーマル',
    '17': '九州そら-セクシー',
    '18': '九州そら-ツンツン',
    '19': '九州そら-ささやき',
}

name_to_speaker_id = {
    b: a for a, b in speaker_id_to_name.items()
}


DEFAULT_SPEAKER_ID = os.environ.get(
    'VOICEVOX_DEFAULT_SPEAKER_ID', '2')
if not DEFAULT_SPEAKER_ID.isdigit():
    DEFAULT_SPEAKER_ID = name_to_speaker_id[DEFAULT_SPEAKER_ID]
VOICEVOX_TEXTTOSPEECH_URL = os.environ.get(
    'VOICEVOX_TEXTTOSPEECH_URL', 'localhost')
VOICEVOX_TEXTTOSPEECH_PORT = os.environ.get(
    'VOICEVOX_TEXTTOSPEECH_PORT', 50021)
cache_enabled = os.environ.get(
    'ROS_VOICEVOX_TEXTTOSPEECH_CACHE_ENABLED', True)
cache_enabled = cache_enabled is True \
    or cache_enabled == 'true'  # for launch env tag.


def determine_voice_name(voice_name):
    if len(voice_name) == 0:
        speaker_id = DEFAULT_SPEAKER_ID
    else:
        if voice_name.isdigit():
            if voice_name in speaker_id_to_name:
                speaker_id = voice_name
            else:
                print(
                    '[Text2Wave] Invalid speaker_id ({}). Use default voice.'
                    .format(speaker_id_to_name[DEFAULT_SPEAKER_ID]))
                speaker_id = DEFAULT_SPEAKER_ID
        else:
            candidates = list(filter(
                lambda name: name.startswith(voice_name),
                name_to_speaker_id))
            if candidates:
                speaker_id = name_to_speaker_id[candidates[0]]
            else:
                print('[Text2Wave] Invalid voice_name ({}). Use default voice.'
                      .format(speaker_id_to_name[DEFAULT_SPEAKER_ID]))
                speaker_id = DEFAULT_SPEAKER_ID
    print('[Text2Wave] Speak using voice_name ({})..'.format(
        speaker_id_to_name[speaker_id]))
    return speaker_id


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

async def request_synthesis(
        sentence, output_path, speaker_id='1'):
    async with Client() as client:
        audio_query = await client.create_audio_query(sentence, speaker=speaker_id)
        print(audio_query)
        with open(output_path, "wb") as f:
            f.write(await audio_query.synthesis(speaker=speaker_id))
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='')
    parser.add_argument('-eval', '--evaluate')
    parser.add_argument('-o', '--output')
    parser.add_argument('text')
    args = parser.parse_args()

    with open(args.text, 'rb') as f:
        speech_text = convert_to_str(f.readline())

    print('args')
    print(args)
    speaker_id = args.evaluate.lstrip('(').rstrip(')')
    print('id')
    print(speaker_id)

    asyncio.run(request_synthesis(speech_text,
                                  args.output,
                                  speaker_id))
