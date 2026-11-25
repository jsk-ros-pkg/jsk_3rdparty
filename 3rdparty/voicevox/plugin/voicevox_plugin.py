#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""sound_play plugin that synthesizes speech with a VOICEVOX engine."""

import hashlib
import http.client
import json
import os
import tempfile
import threading
import time
import urllib.parse

import rospkg
import rospy

from sound_play.sound_play_plugin import SoundPlayPlugin


def _env_bool(name, default=True):
    value = os.environ.get(name)
    if value is None:
        return default
    return value.lower() in ('1', 'true', 'yes')


class VoicevoxPlugin(SoundPlayPlugin):

    def __init__(self):
        super(VoicevoxPlugin, self).__init__()
        self._host = os.environ.get('VOICEVOX_TEXTTOSPEECH_URL', 'localhost')
        self._port = int(os.environ.get('VOICEVOX_TEXTTOSPEECH_PORT', '50021'))
        self._default_speaker = str(
            os.environ.get('VOICEVOX_DEFAULT_SPEAKER_ID', 2))
        self._timeout = float(
            os.environ.get('VOICEVOX_TEXTTOSPEECH_TIMEOUT', '90'))
        self._startup_timeout = float(
            os.environ.get('VOICEVOX_STARTUP_TIMEOUT', '300'))
        self._retry_interval = float(
            os.environ.get('VOICEVOX_RETRY_INTERVAL', '1'))
        self._cache_enabled = _env_bool(
            'ROS_VOICEVOX_TEXTTOSPEECH_CACHE_ENABLED', True)
        self._cache_dir = os.path.join(rospkg.get_ros_home(), 'voicevox')
        if self._cache_enabled:
            os.makedirs(self._cache_dir, exist_ok=True)
        self._conn = None
        self._lock = threading.Lock()
        self._speakers = None
        rospy.loginfo('[voicevox_plugin] server=%s:%d default_speaker=%s '
                      'cache=%s' % (self._host, self._port,
                                    self._default_speaker,
                                    self._cache_dir if self._cache_enabled
                                    else 'disabled'))

    def _request(self, method, path, body=None, headers=None):
        # wait while the engine can not be reached, like request_synthesis.py
        deadline = time.time() + self._startup_timeout
        retry = 0
        while True:
            try:
                return self._request_once(method, path, body, headers)
            except (http.client.HTTPException, OSError) as e:
                if time.time() >= deadline:
                    raise
                if retry % 10 == 0:
                    rospy.logwarn(
                        '[voicevox_plugin] waiting for VOICEVOX engine at '
                        '%s:%d: %s' % (self._host, self._port, e))
                retry += 1
                time.sleep(self._retry_interval)

    def _request_once(self, method, path, body, headers):
        for attempt in (0, 1):
            try:
                # the engine may close the connection (Connection: close),
                # then reconnect to set the timeout for synthesis again
                if self._conn is None or self._conn.sock is None:
                    # short timeout for connecting, long one for synthesis
                    self._conn = http.client.HTTPConnection(
                        self._host, self._port, timeout=5)
                    self._conn.connect()
                    self._conn.sock.settimeout(self._timeout)
                self._conn.request(method, path, body=body,
                                   headers=headers or {})
                res = self._conn.getresponse()
                data = res.read()
                if res.status != 200:
                    raise RuntimeError('HTTP %d for %s: %s' % (
                        res.status, path.split('?')[0], data[:200]))
                return data
            except (http.client.HTTPException, OSError):
                try:
                    self._conn.close()
                except Exception:
                    pass
                self._conn = None
                if attempt == 1:
                    raise

    def _fetch_speakers(self):
        # share speakers.json with text2wave (request_synthesis.py), so that
        # a named voice is resolved without the engine after a restart
        speakers_cache = os.path.join(self._cache_dir, 'speakers.json')
        if os.path.exists(speakers_cache):
            with open(speakers_cache, encoding='utf-8') as f:
                return json.load(f)
        data = self._request('GET', '/speakers')
        speakers = {}
        for speaker in json.loads(data.decode('utf-8')):
            for style in speaker['styles']:
                speakers[str(style['id'])] = '%s-%s' % (
                    speaker['name'], style['name'])
        os.makedirs(self._cache_dir, exist_ok=True)
        with open(speakers_cache, 'w', encoding='utf-8') as f:
            json.dump(speakers, f, ensure_ascii=False)
        return speakers

    def _resolve_speaker(self, voice):
        if voice is None:
            return self._default_speaker
        name = voice.strip().lstrip('(').rstrip(')')
        if name == '':
            return self._default_speaker
        if name.isdigit():
            return name
        if self._speakers is None:
            try:
                self._speakers = self._fetch_speakers()
            except Exception as e:
                rospy.logwarn('[voicevox_plugin] cannot fetch speakers: %s' % e)
                return self._default_speaker
        # first prefix match in server order, like request_synthesis.py
        for speaker_id, full_name in self._speakers.items():
            if full_name.startswith(name):
                return speaker_id
        return self._default_speaker

    def _cache_path(self, text, speaker_id):
        digest = hashlib.md5(text.encode('utf-8')).hexdigest()
        return os.path.join(self._cache_dir,
                            '%s--%s.wav' % (digest, speaker_id))

    def sound_play_say_plugin(self, text, voice):
        if isinstance(text, bytes):
            text = text.decode('utf-8')
        start = time.time()
        with self._lock:
            speaker_id = self._resolve_speaker(voice)
            cache_path = self._cache_path(text, speaker_id) \
                if self._cache_enabled else None
            if cache_path is not None and os.path.exists(cache_path):
                rospy.logdebug('[voicevox_plugin] cached (%.3fs): %s' % (
                    time.time() - start, text))
                return cache_path
            try:
                query = self._request(
                    'POST', '/audio_query?' + urllib.parse.urlencode(
                        {'text': text, 'speaker': speaker_id}))
                wav = self._request(
                    'POST', '/synthesis?speaker=%s' % speaker_id, body=query,
                    headers={'Content-Type': 'application/json'})
            except Exception as e:
                rospy.logerr('[voicevox_plugin] synthesis failed for %s:%d: %s'
                             % (self._host, self._port, e))
                return None
            if cache_path is not None:
                fd, tmp = tempfile.mkstemp(
                    prefix='.tmp', suffix='.wav', dir=self._cache_dir)
                with os.fdopen(fd, 'wb') as f:
                    f.write(wav)
                os.replace(tmp, cache_path)
                wavfilename = cache_path
            else:
                fd, wavfilename = tempfile.mkstemp(
                    prefix='sound_play', suffix='.wav')
                with os.fdopen(fd, 'wb') as f:
                    f.write(wav)
            rospy.loginfo('[voicevox_plugin] synthesized in %.3fs (speaker %s)'
                          ': %s' % (time.time() - start, speaker_id, text))
            return wavfilename
