import queue
import threading
import time

import numpy as np


class GoogleCloudStreamingRecognizer(object):

    def __init__(self, sample_rate=16000, language_code="ja-JP", model="",
                 endpoint_stable_s=1.0):
        from google.cloud import speech_v1 as speech
        self._speech = speech
        self._client = speech.SpeechClient()
        self._sample_rate = sample_rate
        self._endpoint_stable_s = endpoint_stable_s

        cfg_kwargs = dict(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=sample_rate,
            language_code=language_code,
            max_alternatives=1,
            enable_automatic_punctuation=True,
        )
        if model:
            cfg_kwargs["model"] = model
        self._config = speech.RecognitionConfig(**cfg_kwargs)
        self._streaming_config = speech.StreamingRecognitionConfig(
            config=self._config,
            interim_results=True,
        )
        self._lock = threading.Lock()
        self._init_stream_state()

    def _init_stream_state(self):
        self._audio_q = queue.Queue()
        self._latest_text = ""
        self._committed_text = ""
        self._endpoint = False
        self._last_text_change = time.time()
        self._stop_evt = threading.Event()
        self._thread = None

    def _start_stream(self):
        self._init_stream_state()
        self._thread = threading.Thread(target=self._stream_thread, daemon=True)
        self._thread.start()

    def stop_stream(self):
        """Close the gRPC session. Next accept_waveform() lazily restarts."""
        with self._lock:
            self._stop_evt.set()
        try:
            self._audio_q.put_nowait(None)
        except Exception:
            pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def _audio_gen(self):
        while not self._stop_evt.is_set():
            try:
                chunk = self._audio_q.get(timeout=0.5)
            except queue.Empty:
                continue
            if chunk is None:
                return
            yield self._speech.StreamingRecognizeRequest(audio_content=chunk)

    def _stream_thread(self):
        try:
            responses = self._client.streaming_recognize(
                self._streaming_config, self._audio_gen())
            for response in responses:
                if self._stop_evt.is_set():
                    break
                for result in response.results:
                    if not result.alternatives:
                        continue
                    text = result.alternatives[0].transcript
                    with self._lock:
                        if text != self._latest_text:
                            self._latest_text = text
                            self._last_text_change = time.time()
                        if (result.is_final and text.strip()
                                and text != self._committed_text):
                            self._endpoint = True
        except Exception:
            # 305s session limit / network glitches — restart lazily on the
            # next accept_waveform().
            pass

    def create_stream(self):
        self._start_stream()
        return self

    def accept_waveform(self, sample_rate, arr_float32):
        if self._thread is None or not self._thread.is_alive():
            self._start_stream()
        pcm = (np.clip(arr_float32, -1, 1) * 32767).astype(np.int16).tobytes()
        self._audio_q.put(pcm)

        with self._lock:
            if (self._latest_text.strip()
                    and self._latest_text != self._committed_text
                    and (time.time() - self._last_text_change) >= self._endpoint_stable_s):
                self._endpoint = True

    def is_ready(self, stream): return False
    def decode_stream(self, stream): pass
    def decode_streams(self, streams): pass

    def get_result(self, stream):
        with self._lock:
            latest = self._latest_text
            if latest == self._committed_text:
                return ""
            return latest

    def is_endpoint(self, stream):
        with self._lock:
            return self._endpoint

    def reset(self, stream):
        # Don't tear the gRPC stream down between utterances. Mark the latest
        # text as consumed so late-arriving refinements for the same utterance
        # don't re-fire endpoint.
        with self._lock:
            self._committed_text = self._latest_text
            self._endpoint = False
            self._last_text_change = time.time()

