# file to override recognize_vosk ( https://github.com/Uberi/speech_recognition/blob/3.9.0/speech_recognition/__init__.py#L1711 )  # noqa: E501
# we need this to use vosk model anywhere

import json
import logging
import os
import os.path as osp

from ament_index_python.packages import get_package_share_directory
from ros_speech_recognition.install_trained_data import default_data_directory
from speech_recognition import AudioData


try:
    from vosk import Model, KaldiRecognizer
except ImportError:
    Model = None
    KaldiRecognizer = None


LOGGER = logging.getLogger(__name__)


def _default_model_path(language):
    model_names = {
        'en-US': 'vosk-model-small-en-us-0.15',
        'ja': 'vosk-model-small-ja-0.22',
        'ja-JP': 'vosk-model-small-ja-0.22',
    }
    if language not in model_names:
        raise ValueError(
            'Unsupported language {}. Download a model and set '
            'vosk_model_path.'.format(language))

    name = model_names[language]
    search_roots = []
    environment_path = os.environ.get('ROS_SPEECH_RECOGNITION_DATA_DIR')
    if environment_path:
        search_roots.append(environment_path)
    search_roots.append(str(default_data_directory()))
    try:
        search_roots.append(osp.join(
            get_package_share_directory('ros_speech_recognition'),
            'trained_data'))
    except LookupError:
        pass

    for root in search_roots:
        path = osp.join(root, name)
        if osp.isdir(path):
            return path
    raise FileNotFoundError(
        '{} was not found. Run `ros2 run ros_speech_recognition '
        'install_trained_data`.'.format(name))


def recognize_vosk(self, audio_data, model_path=None, language='en-US'):
    if Model is None or KaldiRecognizer is None:
        raise ImportError(
            'vosk is not installed; run `uv sync` before building/running')
    if not isinstance(audio_data, AudioData):
        raise TypeError('audio_data must be an AudioData instance')

    if not hasattr(self, 'vosk_model'):
        if model_path is None:
            model_path = _default_model_path(language)
        LOGGER.info('Loading model from %s', model_path)
        self.vosk_model = Model(model_path)
    recognizer = KaldiRecognizer(self.vosk_model, 16000)

    recognizer.AcceptWaveform(
        audio_data.get_raw_data(convert_rate=16000, convert_width=2))
    return json.loads(recognizer.FinalResult())['text']
