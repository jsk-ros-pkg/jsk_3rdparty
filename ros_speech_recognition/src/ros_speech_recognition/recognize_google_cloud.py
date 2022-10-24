# file to override recognize_google_cloud (https://github.com/Uberi/speech_recognition/blob/3.8.1/speech_recognition/__init__.py)
#
# we need this to pass more config params, like enable_speaker_diarization=True
# see https://cloud.google.com/speech-to-text/docs/multiple-voices

import speech_recognition as SR
from speech_recognition import *

class RecognizerEx(SR.Recognizer):
    def recognize_google_cloud(self, audio_data, credentials_json=None, language="en-US", preferred_phrases=None, show_all=False, user_config = {}):
        """
        Performs speech recognition on ``audio_data`` (an ``AudioData`` instance), using the Google Cloud Speech API.

        This function requires a Google Cloud Platform account; see the `Google Cloud Speech API Quickstart <https://cloud.google.com/speech/docs/getting-started>`__ for details and instructions. Basically, create a project, enable billing for the project, enable the Google Cloud Speech API for the project, and set up Service Account Key credentials for the project. The result is a JSON file containing the API credentials. The text content of this JSON file is specified by ``credentials_json``. If not specified, the library will try to automatically `find the default API credentials JSON file <https://developers.google.com/identity/protocols/application-default-credentials>`__.

        The recognition language is determined by ``language``, which is a BCP-47 language tag like ``"en-US"`` (US English). A list of supported language tags can be found in the `Google Cloud Speech API documentation <https://cloud.google.com/speech/docs/languages>`__.

        If ``preferred_phrases`` is an iterable of phrase strings, those given phrases will be more likely to be recognized over similar-sounding alternatives. This is useful for things like keyword/command recognition or adding new phrases that aren't in Google's vocabulary. Note that the API imposes certain `restrictions on the list of phrase strings <https://cloud.google.com/speech/limits#content>`__.

        Returns the most likely transcription if ``show_all`` is False (the default). Otherwise, returns the raw API response as a JSON dictionary.

        Raises a ``speech_recognition.UnknownValueError`` exception if the speech is unintelligible. Raises a ``speech_recognition.RequestError`` exception if the speech recognition operation failed, if the credentials aren't valid, or if there is no Internet connection.
        """
        assert isinstance(audio_data, AudioData), "``audio_data`` must be audio data"
        if credentials_json is not None:
            try: json.loads(credentials_json)
            except Exception: raise AssertionError("``credentials_json`` must be ``None`` or a valid JSON string")
        assert isinstance(language, str), "``language`` must be a string"
        assert preferred_phrases is None or all(isinstance(preferred_phrases, (type(""), type(u""))) for preferred_phrases in preferred_phrases), "``preferred_phrases`` must be a list of strings"

        # See https://cloud.google.com/speech/reference/rest/v1/RecognitionConfig
        flac_data = audio_data.get_flac_data(
            convert_rate=None if 8000 <= audio_data.sample_rate <= 48000 else max(8000, min(audio_data.sample_rate, 48000)),  # audio sample rate must be between 8 kHz and 48 kHz inclusive - clamp sample rate into this range
            convert_width=2  # audio samples must be 16-bit
        )

        try:
            from oauth2client.client import GoogleCredentials
            from googleapiclient.discovery import build
            import googleapiclient.errors

            # cannot simply use 'http = httplib2.Http(timeout=self.operation_timeout)'
            # because discovery.build() says 'Arguments http and credentials are mutually exclusive'
            import socket
            import googleapiclient.http
            if self.operation_timeout and socket.getdefaulttimeout() is None:
                # override constant (used by googleapiclient.http.build_http())
                googleapiclient.http.DEFAULT_HTTP_TIMEOUT_SEC = self.operation_timeout

            if credentials_json is None:
                api_credentials = GoogleCredentials.get_application_default()
            else:
                # the credentials can only be read from a file, so we'll make a temp file and write in the contents to work around that
                with PortableNamedTemporaryFile("w") as f:
                    f.write(credentials_json)
                    f.flush()
                    api_credentials = GoogleCredentials.from_stream(f.name)

            speech_service = build("speech", "v1", credentials=api_credentials, cache_discovery=False)
        except ImportError:
            raise RequestError("missing google-api-python-client module: ensure that google-api-python-client is set up correctly.")

        speech_config = {"encoding": "FLAC", "sampleRateHertz": audio_data.sample_rate, "languageCode": language}

        ###################################
        ##  Support user defined configs ##
        ###################################
        speech_config.update(user_config)
        ###################################
        ##                               ##
        ###################################

        if preferred_phrases is not None:
            speech_config["speechContext"] = {"phrases": preferred_phrases}
        if show_all:
            speech_config["enableWordTimeOffsets"] = True  # some useful extra options for when we want all the output
        request = speech_service.speech().recognize(body={"audio": {"content": base64.b64encode(flac_data).decode("utf8")}, "config": speech_config})

        try:
            response = request.execute()
        except googleapiclient.errors.HttpError as e:
            raise RequestError(e)
        except URLError as e:
            raise RequestError("recognition connection failed: {0}".format(e.reason))

        if show_all: return response
        if "results" not in response or len(response["results"]) == 0: raise UnknownValueError()
        transcript = ""
        for result in response["results"]:
            if 'diarizationConfig' in speech_config and \
               speech_config['diarizationConfig']['enableSpeakerDiarization'] == True:
                # when diariazationConfig is true, use words with speakerTag:
                speakerTag = None
                for word in result["alternatives"][0]["words"]:
                    if 'speakerTag' in word:
                        if speakerTag != word['speakerTag']:
                            speakerTag = word['speakerTag']
                            transcript += "[{}]".format(speakerTag)
                        transcript += ' ' + word['word']
            elif "transcript" in result["alternatives"][0]:
                print("trasncript?")
                transcript += result["alternatives"][0]["transcript"].strip() + " "

        return transcript
