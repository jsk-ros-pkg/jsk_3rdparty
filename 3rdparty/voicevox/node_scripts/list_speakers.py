#!/usr/bin/env python
# -*- coding:utf-8 -*-

from voicevox import Client
import asyncio
import os

VOICEVOX_TEXTTOSPEECH_URL = os.environ.get(
    'VOICEVOX_TEXTTOSPEECH_URL', 'localhost')
VOICEVOX_TEXTTOSPEECH_PORT = os.environ.get(
    'VOICEVOX_TEXTTOSPEECH_PORT', '50021')

async def main():
    async with Client(base_url='http://'+VOICEVOX_TEXTTOSPEECH_URL+':'+VOICEVOX_TEXTTOSPEECH_PORT) as client:
        # check core
        for version in await client.fetch_core_versions():
            print("Core version: {}".format(version))
        # check engine
        engine_version = await client.fetch_engine_version()
        print("Engine version: {}".format(engine_version))
        # check device
        for device in await client.http.request("GET", "/supported_devices"):
            print("Device: {}".format(device))
        # check speaker
        speaker_id_to_name = {}
        for speaker in await client.fetch_speakers():
            print(speaker.uuid, speaker.name, speaker.supported_features.permitted_synthesis_morphing)
            for styles in speaker.styles:
                speaker_id_to_name[styles.id] = speaker.name + '-' + styles.name
        for id, name in sorted(speaker_id_to_name.items(), key=lambda x: int(x[0])):
            print("{} : {}".format(id, name))

if __name__ == "__main__":
    asyncio.run(main())
