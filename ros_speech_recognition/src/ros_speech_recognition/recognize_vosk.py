# file to override recognize_vosk ( https://github.com/Uberi/speech_recognition/blob/3.9.0/speech_recognition/__init__.py#L1711 )
# we need this to use vosk model anywhere

from speech_recognition import AudioData
from ros_speech_recognition.recognize_google_cloud import RecognizerEx
try:
    from vosk import Model, KaldiRecognizer
except ImportError as e:
    bold_red = "\x1b[31;1m"
    reset = "\x1b[0m"
    print(bold_red + str(e)+'\nvsok is not installed try "pip install vosk"' + reset)
import json
import os.path as osp
import rospkg
import rospy

def recognize_vosk(self, audio_data, model_path=None, language='en-US'):

    assert isinstance(audio_data, AudioData), "Data must be audio data"

    if not hasattr(self, 'vosk_model'):
        if model_path is None:
            PKG = 'ros_speech_recognition'
            rp = rospkg.RosPack()
            data_path = osp.join(rp.get_path(PKG), 'trained_data')
            if language == 'en-US':
                model_path = osp.join(data_path, 'vosk-model-small-en-us-0.15')
            elif language == 'ja':
                model_path = osp.join(data_path, 'vosk-model-small-ja-0.22')
            else:
                rospy.logerr("Unsupported language: {0}.\n Please download the model from https://alphacephei.com/vosk/models and specify its path as 'vosk_model_path'.".format(language))
                exit (1)
        rospy.loginfo("Loading model from {}".format(model_path))
        self.vosk_model = Model(model_path)
    rec = KaldiRecognizer(self.vosk_model, 16000);

    rec.AcceptWaveform(audio_data.get_raw_data(convert_rate=16000, convert_width=2));
    finalRecognition = rec.FinalResult()
    text = json.loads(finalRecognition)['text']
    return text

RecognizerEx.recognize_vosk = recognize_vosk
