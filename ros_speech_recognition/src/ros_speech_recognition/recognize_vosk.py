# file to override recognize_vosk
# we need this to use vosk model anywhere

from speech_recognition import *
from ros_speech_recognition.recognize_google_cloud import RecognizerEx
from vosk import Model, KaldiRecognizer
import json

def recognize_vosk(self, audio_data, model_path=None, language='en'):

    assert isinstance(audio_data, AudioData), "Data must be audio data"

    if not hasattr(self, 'vosk_model'):
        if model_path is not None:
            self.vosk_model = Model(model_path)
        else:
            print("Please download the model from https://alphacephei.com/vosk/models and specify its path as 'vosk_model_path'.")
            exit (1)

    rec = KaldiRecognizer(self.vosk_model, 16000);

    rec.AcceptWaveform(audio_data.get_raw_data(convert_rate=16000, convert_width=2));
    finalRecognition = rec.FinalResult()
    text = json.loads(finalRecognition)['text']
    return text

RecognizerEx.recognize_vosk = recognize_vosk
