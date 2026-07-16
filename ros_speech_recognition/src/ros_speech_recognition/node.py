#!/usr/bin/env python3

import array
import json
import os
import sys
from threading import Lock
import time

from action_msgs.msg import GoalStatus
from action_msgs.msg import GoalStatusArray
from audio_common_msgs.msg import AudioData
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_speech_recognition.recognize_google_cloud import RecognizerEx
from ros_speech_recognition.recognize_vosk import recognize_vosk
from sound_play_msgs.action import SoundRequest as SoundRequestAction
from sound_play_msgs.msg import SoundRequest
import speech_recognition as sr
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from speech_recognition_msgs.srv import SpeechRecognition
from std_srvs.srv import Empty


ENGINES = (
    'Google',
    'GoogleCloud',
    'Sphinx',
    'Wit',
    'Bing',
    'Houndify',
    'IBM',
    'Vosk',
)

FLOAT_PARAMETERS = {
    'duration',
    'energy_threshold',
    'dynamic_energy_adjustment_damping',
    'dynamic_energy_ratio',
    'pause_threshold',
    'operation_timeout',
    'listen_timeout',
    'phrase_time_limit',
    'phrase_threshold',
    'non_speaking_duration',
    'tts_tolerance',
}

RecognizerEx.recognize_vosk = recognize_vosk


def array_to_bytes(value):
    return value.tobytes()


def resolve_sound_signal_path(path):
    if os.path.exists(path):
        return path

    attempted_paths = [path]
    base, extension = os.path.splitext(path)
    if extension == '.ogg':
        oga_path = base + '.oga'
        attempted_paths.append(oga_path)
        if os.path.exists(oga_path):
            return oga_path

    raise FileNotFoundError(
        'Sound signal file not found: {}'.format(' or '.join(attempted_paths)))


class ROSAudio(sr.AudioSource):
    def __init__(self, node, topic_name='audio', depth=16, n_channel=1,
                 sample_rate=16000, chunk_size=1024, buffer_size=10240,
                 callback_group=None):
        if buffer_size <= chunk_size:
            raise ValueError('buffer_size must be greater than chunk_size')
        if depth not in (8, 16, 32):
            raise ValueError('depth must be 8, 16 or 32')
        if n_channel < 1:
            raise ValueError('n_channel must be greater than zero')

        self.node = node
        self.topic_name = topic_name
        self.buffer_size = buffer_size
        self.SAMPLE_WIDTH = depth // 8
        self.SAMPLE_RATE = sample_rate
        self.CHUNK = chunk_size
        self.n_channel = n_channel
        self.callback_group = callback_group
        self.stream = None

    def open(self):  # noqa: A003
        if self.stream is not None:
            self.stream.close()
        self.stream = ROSAudio.AudioStream(
            self.node,
            self.topic_name,
            self.buffer_size,
            depth=self.SAMPLE_WIDTH * 8,
            n_channel=self.n_channel,
            callback_group=self.callback_group,
        )
        return self

    def close(self):
        if self.stream is not None:
            self.stream.close()
            self.stream = None

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    class AudioStream:
        def __init__(self, node, topic_name, buffer_size=10240, depth=16,
                     n_channel=1, target_channel=0, callback_group=None):
            self.node = node
            self.buffer_size = buffer_size
            self.lock = Lock()
            self.buffer = bytes()
            self.closed = False
            self.depth = depth
            self.n_channel = n_channel
            self.target_channel = min(n_channel - 1, max(0, target_channel))
            self.sub_audio = node.create_subscription(
                AudioData,
                topic_name,
                self.audio_cb,
                10,
                callback_group=callback_group,
            )
            self.type_code = {
                array.array(code).itemsize: code for code in ('b', 'h', 'i')
            }

        def read_once(self, size):
            with self.lock:
                data = self.buffer[:size]
                self.buffer = self.buffer[size:]
                return data

        def read(self, size):
            while rclpy.ok(context=self.node.context) and not self.closed:
                with self.lock:
                    if len(self.buffer) >= size:
                        break
                time.sleep(0.001)
            return self.read_once(size)

        def close(self):
            if self.closed:
                return
            self.closed = True
            self.node.destroy_subscription(self.sub_audio)
            with self.lock:
                self.buffer = bytes()

        def audio_cb(self, msg):
            sample_width = self.depth // 8
            if sample_width not in self.type_code:
                self.node.get_logger().error(
                    'Unsupported audio sample width: {}'.format(sample_width))
                return

            dtype = self.type_code[sample_width]
            data = array.array(dtype, bytes(msg.data))
            channel_data = data[self.target_channel::self.n_channel]
            with self.lock:
                self.buffer += array_to_bytes(channel_data)
                overflow = len(self.buffer) - self.buffer_size
                if overflow > 0:
                    self.buffer = self.buffer[overflow:]


class SpeechRecognitionNode(Node):
    _DYNAMIC_PARAMETERS = {
        'language',
        'engine',
        'energy_threshold',
        'dynamic_energy_threshold',
        'dynamic_energy_adjustment_damping',
        'dynamic_energy_ratio',
        'pause_threshold',
        'operation_timeout',
        'listen_timeout',
        'phrase_time_limit',
        'phrase_threshold',
        'non_speaking_duration',
        'google_key',
        'google_cloud_credentials_json',
        'google_cloud_preferred_phrases',
        'diarization_config',
        'bing_key',
        'vosk_model_path',
    }

    def __init__(self):
        super().__init__('speech_recognition')
        self.callback_group = ReentrantCallbackGroup()
        self._declare_parameters()

        self.default_duration = self._parameter('duration')
        self.recognizer = RecognizerEx()
        self.engine = None
        self.args = {}
        self._apply_recognizer_parameters(self._dynamic_parameter_values())

        self.audio = ROSAudio(
            self,
            topic_name=self._parameter('audio_topic'),
            depth=self._parameter('depth'),
            n_channel=self._parameter('n_channel'),
            sample_rate=self._parameter('sample_rate'),
            chunk_size=self._parameter('chunk_size'),
            buffer_size=self._parameter('buffer_size'),
            callback_group=self.callback_group,
        )

        self.enable_sound_effect = self._parameter('enable_sound_effect')
        self.sound_client = None
        self._sound_server_warning_logged = False
        if self.enable_sound_effect:
            self.sound_client = ActionClient(
                self,
                SoundRequestAction,
                self._parameter('sound_action_name'),
                callback_group=self.callback_group,
            )
        self.signals = self._load_sound_signals()

        self.self_cancellation = self._parameter('self_cancellation')
        self.tts_tolerance = self._parameter('tts_tolerance')
        self.tts_status = {}
        self.last_tts = None
        self.is_canceling = False
        self.tts_status_subscriptions = []
        if self.self_cancellation:
            for action_name in self._parameter('tts_action_names'):
                self.tts_status[action_name] = False
                topic = action_name.rstrip('/') + '/_action/status'
                subscription = self.create_subscription(
                    GoalStatusArray,
                    topic,
                    lambda msg, name=action_name: self._tts_status_cb(msg, name),
                    10,
                    callback_group=self.callback_group,
                )
                self.tts_status_subscriptions.append(subscription)
            self.tts_timer = self.create_timer(
                0.1, self._tts_timer_cb, callback_group=self.callback_group)

        self.stop_fn = None
        self.continuous = self._parameter('continuous')
        self.auto_start = self._parameter('auto_start')
        self.enable_audio_cb = self.auto_start
        if self.continuous:
            self.get_logger().info('Enabled continuous mode')
            self.get_logger().info('Auto start: {}'.format(self.auto_start))
            self.publisher = self.create_publisher(
                SpeechRecognitionCandidates,
                self._parameter('voice_topic'),
                1,
            )
            self.start_service = self.create_service(
                Empty,
                'speech_recognition/start',
                self._start_service_cb,
                callback_group=self.callback_group,
            )
            self.stop_service = self.create_service(
                Empty,
                'speech_recognition/stop',
                self._stop_service_cb,
                callback_group=self.callback_group,
            )
            if self.auto_start:
                self.startup_timer = self.create_timer(
                    0.1,
                    self._start_from_timer,
                    callback_group=self.callback_group,
                )
        else:
            self.get_logger().info(
                'Disabled continuous mode; waiting for speech_recognition service')
            self.recognition_service = self.create_service(
                SpeechRecognition,
                'speech_recognition',
                self._recognition_service_cb,
                callback_group=self.callback_group,
            )

        self.parameter_callback = self.add_on_set_parameters_callback(
            self._on_set_parameters)

    def _declare_parameters(self):
        parameters = {
            'duration': 10.0,
            'language': 'en-US',
            'engine': 'Google',
            'energy_threshold': 300.0,
            'dynamic_energy_threshold': True,
            'dynamic_energy_adjustment_damping': 0.15,
            'dynamic_energy_ratio': 1.5,
            'pause_threshold': 0.8,
            'operation_timeout': 0.0,
            'listen_timeout': 0.0,
            'phrase_time_limit': 10.0,
            'phrase_threshold': 0.3,
            'non_speaking_duration': 0.5,
            'audio_topic': 'audio',
            'voice_topic': 'speech_to_text',
            'depth': 16,
            'n_channel': 1,
            'sample_rate': 16000,
            'chunk_size': 1024,
            'buffer_size': 10240,
            'enable_sound_effect': True,
            'sound_action_name': 'sound_play',
            'start_signal':
                '/usr/share/sounds/freedesktop/stereo/bell.ogg',
            'recognized_signal':
                '/usr/share/sounds/freedesktop/stereo/message.ogg',
            'success_signal':
                '/usr/share/sounds/freedesktop/stereo/message-new-instant.ogg',
            'timeout_signal':
                '/usr/share/sounds/freedesktop/stereo/network-connectivity-lost.ogg',
            'self_cancellation': True,
            'tts_tolerance': 1.0,
            'tts_action_names': ['sound_play'],
            'continuous': False,
            'auto_start': True,
            'google_key': '',
            'google_cloud_credentials_json': '',
            'google_cloud_preferred_phrases': [''],
            'diarization_config': '{}',
            'bing_key': '',
            'vosk_model_path': '',
        }
        dynamic_descriptor = ParameterDescriptor(dynamic_typing=True)
        for name, default in parameters.items():
            descriptor = dynamic_descriptor if name in FLOAT_PARAMETERS else None
            self.declare_parameter(name, default, descriptor=descriptor)

    def _parameter(self, name):
        value = self.get_parameter(name).value
        if name in FLOAT_PARAMETERS:
            return float(value)
        return value

    def _dynamic_parameter_values(self):
        return {name: self._parameter(name) for name in self._DYNAMIC_PARAMETERS}

    def _load_sound_signals(self):
        paths = {
            'start': self._parameter('start_signal'),
            'recognized': self._parameter('recognized_signal'),
            'success': self._parameter('success_signal'),
            'timeout': self._parameter('timeout_signal'),
        }
        if not self.enable_sound_effect:
            return paths

        resolved = {}
        for key, path in paths.items():
            try:
                resolved[key] = resolve_sound_signal_path(path)
            except FileNotFoundError as error:
                self.get_logger().warning(str(error))
                resolved[key] = None
        return resolved

    def _apply_recognizer_parameters(self, values):
        if self.engine != values['engine']:
            self.args = {}
        self.engine = values['engine']
        self.language = values['language']
        self.dynamic_energy_threshold = values['dynamic_energy_threshold']
        self.recognizer.dynamic_energy_threshold = self.dynamic_energy_threshold
        if not self.dynamic_energy_threshold:
            self.recognizer.energy_threshold = values['energy_threshold']
        self.recognizer.dynamic_energy_adjustment_damping = values[
            'dynamic_energy_adjustment_damping']
        self.recognizer.dynamic_energy_ratio = values['dynamic_energy_ratio']
        self.listen_timeout = values['listen_timeout'] or None
        self.phrase_time_limit = values['phrase_time_limit'] or None
        self.recognizer.operation_timeout = values['operation_timeout'] or None
        self.recognizer.pause_threshold = values['pause_threshold']
        self.recognizer.non_speaking_duration = values[
            'non_speaking_duration']
        self.recognizer.phrase_threshold = values['phrase_threshold']

    def _validate_dynamic_parameters(self, values):
        if values['engine'] not in ENGINES:
            return 'engine must be one of {}'.format(', '.join(ENGINES))
        if values['energy_threshold'] <= 0.0:
            return 'energy_threshold must be greater than zero'
        if not 0.0 <= values['dynamic_energy_adjustment_damping'] <= 1.0:
            return 'dynamic_energy_adjustment_damping must be between 0 and 1'
        if values['dynamic_energy_ratio'] < 1.0:
            return 'dynamic_energy_ratio must be at least 1'
        if values['pause_threshold'] < values['non_speaking_duration']:
            return 'pause_threshold must be at least non_speaking_duration'
        for name in ('operation_timeout', 'listen_timeout', 'phrase_time_limit'):
            if values[name] < 0.0:
                return '{} must not be negative'.format(name)
        if values['phrase_threshold'] <= 0.0:
            return 'phrase_threshold must be greater than zero'
        return None

    def _on_set_parameters(self, parameters):
        unsupported = [
            parameter.name for parameter in parameters
            if parameter.name not in self._DYNAMIC_PARAMETERS
        ]
        if unsupported:
            return SetParametersResult(
                successful=False,
                reason='These parameters require a node restart: {}'.format(
                    ', '.join(unsupported)),
            )

        values = self._dynamic_parameter_values()
        reset_args = False
        for parameter in parameters:
            values[parameter.name] = parameter.value
            if parameter.name in {
                    'engine', 'language', 'google_key',
                    'google_cloud_credentials_json',
                    'google_cloud_preferred_phrases', 'diarization_config',
                    'bing_key', 'vosk_model_path'}:
                reset_args = True

        reason = self._validate_dynamic_parameters(values)
        if reason:
            return SetParametersResult(successful=False, reason=reason)
        if reset_args:
            self.args = {}
        self._apply_recognizer_parameters(values)
        return SetParametersResult(successful=True)

    def play_sound(self, key):
        if self.sound_client is None or self.signals.get(key) is None:
            return
        if not self.sound_client.server_is_ready():
            if not self._sound_server_warning_logged:
                self.get_logger().warning(
                    'sound_play action is not ready; sound effects are skipped')
                self._sound_server_warning_logged = True
            return

        request = SoundRequest()
        request.sound = SoundRequest.PLAY_FILE
        request.command = SoundRequest.PLAY_ONCE
        request.volume = 1.0
        request.arg = self.signals[key]
        goal = SoundRequestAction.Goal()
        goal.sound_request = request
        self.sound_client.send_goal_async(goal)

    def recognize(self, audio):
        if not self.args:
            self.args = self._recognizer_arguments()

        functions = {
            'Google': 'recognize_google',
            'GoogleCloud': 'recognize_google_cloud',
            'Sphinx': 'recognize_sphinx',
            'Wit': 'recognize_wit',
            'Bing': 'recognize_bing',
            'Houndify': 'recognize_houndify',
            'IBM': 'recognize_ibm',
            'Vosk': 'recognize_vosk',
        }
        function_name = functions[self.engine]
        recognize = getattr(self.recognizer, function_name, None)
        if recognize is None:
            raise sr.RequestError(
                '{} is not supported by the installed SpeechRecognition '
                'version'.format(self.engine))
        return recognize(
            audio_data=audio, language=self.language, **self.args)

    def _recognizer_arguments(self):
        if self.engine == 'Google':
            key = self._parameter('google_key') or None
            return {'key': key, 'show_all': True}
        if self.engine == 'GoogleCloud':
            credentials_path = self._parameter(
                'google_cloud_credentials_json')
            credentials_json = None
            if credentials_path:
                with open(credentials_path, encoding='utf-8') as stream:
                    credentials_json = stream.read()
            phrases = [
                phrase for phrase in
                self._parameter('google_cloud_preferred_phrases') if phrase
            ]
            arguments = {
                'credentials_json': credentials_json,
                'preferred_phrases': phrases or None,
            }
            diarization_config = json.loads(
                self._parameter('diarization_config'))
            if diarization_config:
                arguments['user_config'] = {
                    'diarizationConfig': diarization_config}
            return arguments
        if self.engine == 'Bing':
            return {'key': self._parameter('bing_key')}
        if self.engine == 'Vosk':
            return {'model_path': self._parameter('vosk_model_path') or None}
        return {}

    @staticmethod
    def _extract_result(result):
        if isinstance(result, str):
            return result, 1.0
        alternatives = result.get('alternative', []) if result else []
        if not alternatives:
            raise sr.UnknownValueError()
        best = alternatives[0]
        return best.get('transcript', ''), float(best.get('confidence', 1.0))

    def audio_cb(self, _recognizer, audio):
        if not self.enable_audio_cb:
            return
        if self.is_canceling:
            self.get_logger().info(
                'Robot is speaking; speech recognition is cancelled')
            return
        try:
            self.get_logger().debug(
                'Waiting for result (sent {} bytes)'.format(
                    len(audio.get_raw_data())))
            result, confidence = self._extract_result(self.recognize(audio))
            if not result:
                return
            self.play_sound('recognized')
            self.get_logger().info('Result: {}'.format(result))
            self.play_sound('success')
            message = SpeechRecognitionCandidates()
            message.transcript = [result]
            message.confidence = [confidence]
            self.publisher.publish(message)
        except sr.UnknownValueError:
            self.get_logger().warning('Speech was not understood')
            self.play_sound('timeout')
        except sr.RequestError as error:
            self.get_logger().error('Failed to recognize: {}'.format(error))
            self.play_sound('timeout')
        except Exception:
            self.get_logger().error(
                'Unexpected recognition error: {}'.format(sys.exc_info()))
            self.play_sound('timeout')

    def start_speech_recognition(self):
        if self.stop_fn is not None:
            return
        if self.dynamic_energy_threshold:
            with self.audio as source:
                self.recognizer.adjust_for_ambient_noise(source)
                self.get_logger().info(
                    'Set minimum energy threshold to {}'.format(
                        self.recognizer.energy_threshold))
        self.play_sound('start')
        self.stop_fn = self.recognizer.listen_in_background(
            self.audio,
            self.audio_cb,
            phrase_time_limit=self.phrase_time_limit,
        )

    def stop_speech_recognition(self):
        if self.stop_fn is None:
            return
        stop_fn = self.stop_fn
        self.stop_fn = None
        stop_fn(wait_for_stop=False)

    def _recognition_service_cb(self, request, response):
        duration = request.duration if request.duration > 0.0 \
            else self.default_duration
        with self.audio as source:
            if self.dynamic_energy_threshold:
                self.recognizer.adjust_for_ambient_noise(source)
                self.get_logger().info(
                    'Set minimum energy threshold to {}'.format(
                        self.recognizer.energy_threshold))
            if not request.quiet:
                self.play_sound('start')

            start_time = time.monotonic()
            while rclpy.ok(context=self.context) and \
                    time.monotonic() - start_time < duration:
                self.get_logger().info('Waiting for speech')
                try:
                    audio = self.recognizer.listen(
                        source,
                        timeout=self.listen_timeout,
                        phrase_time_limit=self.phrase_time_limit,
                    )
                except sr.WaitTimeoutError as error:
                    self.get_logger().warning(str(error))
                    break
                if not request.quiet:
                    self.play_sound('recognized')

                try:
                    result, confidence = self._extract_result(
                        self.recognize(audio))
                    if not result:
                        continue
                    self.get_logger().info('Result: {}'.format(result))
                    if not request.quiet:
                        self.play_sound('success')
                    response.result.transcript = [result]
                    response.result.confidence = [confidence]
                    return response
                except sr.UnknownValueError:
                    self.get_logger().warning('Speech was not understood')
                except sr.RequestError as error:
                    self.get_logger().error(
                        'Failed to recognize: {}'.format(error))
                time.sleep(0.1)

        if not request.quiet:
            self.play_sound('timeout')
        return response

    def _start_from_timer(self):
        self.startup_timer.cancel()
        self.start_speech_recognition()

    def _start_service_cb(self, _request, response):
        self.enable_audio_cb = True
        self.start_speech_recognition()
        self.get_logger().info('Started continuous mode')
        return response

    def _stop_service_cb(self, _request, response):
        self.enable_audio_cb = False
        self.stop_speech_recognition()
        self.get_logger().info('Stopped continuous mode')
        return response

    def _tts_status_cb(self, message, action_name):
        active_states = {
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        }
        self.tts_status[action_name] = any(
            status.status in active_states for status in message.status_list)

    def _tts_timer_cb(self):
        if any(self.tts_status.values()):
            if not self.is_canceling:
                self.get_logger().debug('START CANCELLATION')
            self.is_canceling = True
            self.last_tts = None
            return

        if not self.is_canceling:
            return
        if self.last_tts is None:
            self.last_tts = time.monotonic()
        if time.monotonic() - self.last_tts > self.tts_tolerance:
            self.get_logger().debug('END CANCELLATION')
            self.is_canceling = False

    def shutdown(self):
        self.stop_speech_recognition()


def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecognitionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown()
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
