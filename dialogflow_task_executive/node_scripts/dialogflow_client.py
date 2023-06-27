#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import actionlib
import dialogflow as df
from google.oauth2.service_account import Credentials
from google.protobuf.json_format import MessageToJson
import pprint
try:
    import Queue
except ImportError:
    import queue as Queue
import os
import rospy
import sys
import threading
import uuid

from audio_common_msgs.msg import AudioData
from sound_play.msg import SoundRequest
from sound_play.msg import SoundRequestAction
from sound_play.msg import SoundRequestGoal
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import String
from dialogflow_task_executive.msg import DialogTextAction, DialogTextGoal, DialogTextResult, DialogTextFeedback

from dialogflow_task_executive.msg import DialogResponse


class State(object):
    IDLE = "IDLE"
    SPEAKING = "SPEAKING"
    LISTENING = "LISTENING"
    THINKING = "THINKING"

    def __init__(self, init_state=None):
        self._state = init_state or self.IDLE
        self._last_state = None
        self._last_changed = rospy.Time.now()

    def set(self, state):
        if self._state != state:
            self._last_state = self._state
            self._state = state
            rospy.loginfo(
                "State: {} -> {}".format(self._last_state, self._state))
            self._last_changed = rospy.Time.now()

    @property
    def current(self):
        return self._state

    @property
    def last_state(self):
        return self._last_state

    @property
    def last_changed(self):
        return self._last_changed

    def __eq__(self, state):
        return self._state == state

    def __ne__(self, state):
        return not self.__eq__(state)


class DialogflowBase(object):

    def __init__(self):
        self.session_id = None
        self.language = rospy.get_param("~language", "ja-JP")
        credentials_json = rospy.get_param(
            '~google_cloud_credentials_json', None)
        if credentials_json is None:
            rospy.loginfo("Loading credential json from env")
            # project id for google cloud service
            self.project_id = rospy.get_param("~project_id", None)
            self.session_client = df.SessionsClient()
        else:
            rospy.loginfo("Loading credential json from rosparam")
            credentials = Credentials.from_service_account_file(
                credentials_json
            )
            self.project_id = credentials.project_id
            if rospy.has_param("~project_id") and rospy.get_param("~override_project_id", False):
                self.project_id = rospy.get_param("~project_id")
                rospy.logwarn("override project_id")
                rospy.logwarn("   from : project_id in a credential file {}".format(credentials.project_id))
                rospy.logwarn("     to : project_id stored in rosparam   {}".format(self.project_id))
            self.session_client = df.SessionsClient(
                credentials=credentials
            )
        if self.project_id is None:
            rospy.logerr('project ID is not set')
        else:
            rospy.loginfo('project ID is "{}"'.format(self.project_id))
        self.pub_res = rospy.Publisher(
            "dialog_response", DialogResponse, queue_size=1)
        self.always_publish_result = rospy.get_param(
            "~always_publish_result", False)

    def detect_intent_text(self, data, session):
        query = df.types.QueryInput(
            text=df.types.TextInput(
                text=data, language_code=self.language))
        return self.session_client.detect_intent(
            session=session, query_input=query).query_result

    def make_dialog_msg(self, result):
        msg = DialogResponse()
        msg.header.stamp = rospy.Time.now()
        if result.action == 'input.unknown':
            rospy.logwarn("Unknown action")
        msg.action = result.action

        # check if ROS_PYTHON_VERSION exists in indigo
        if (self.language == 'ja-JP'
            and ("ROS_PYTHON_VERSION" not in os.environ
                 or os.environ["ROS_PYTHON_VERSION"] == "2")):
            msg.query = result.query_text.encode("utf-8")
            msg.response = result.fulfillment_text.encode("utf-8")
        else:
            msg.query = result.query_text
            msg.response = result.fulfillment_text
        msg.fulfilled = result.all_required_params_present
        msg.parameters = MessageToJson(result.parameters)
        msg.speech_score = result.speech_recognition_confidence
        msg.intent_score = result.intent_detection_confidence
        return msg


class DialogflowTextClient(DialogflowBase):

    def __init__(self):
        super(DialogflowTextClient, self).__init__()
        self._as = actionlib.SimpleActionServer("~text_action", DialogTextAction,
                                                execute_cb=self.cb, auto_start=False)
        self._as.start()

    def cb(self, goal):
        feedback = DialogTextFeedback()
        result = DialogTextResult()
        success = False
        try:
            if self.session_id is None:
                self.session_id = str(uuid.uuid1())
                rospy.loginfo(
                    "DialogflowTextClient: Created new session: {}".format(self.session_id))
            session = self.session_client.session_path(
                self.project_id, self.session_id
            )
            df_result = self.detect_intent_text(goal.query, session)
            result.session = session
            result.response = self.make_dialog_msg(df_result)
            success = True
        except Exception as e:
            rospy.logerr(str(e))
            feedback.status = str(e)
            success = False
        finally:
            self._as.publish_feedback(feedback)
            result.done = success
            self._as.set_succeeded(result)
            if df_result and self.always_publish_result:
                self.pub_res.publish(result.response)


class DialogflowAudioClient(DialogflowBase):

    def __init__(self):
        super(DialogflowAudioClient, self).__init__()
        # language for dialogflow
        self.language = rospy.get_param("~language", "ja-JP")

        # use raw audio data if enabled, otherwise use recognized STT data
        self.use_audio = rospy.get_param("~use_audio", False)
        # sample rate of audio data
        self.audio_sample_rate = rospy.get_param("~audio_sample_rate", 16000)

        # use TTS feature
        self.use_tts = rospy.get_param("~use_tts", True)
        self.volume = rospy.get_param('~volume', 1.0)

        # timeout for voice input activation by hotword
        self.timeout = rospy.get_param("~timeout", 10.0)
        # hotwords
        self.enable_hotword = rospy.get_param("~enable_hotword", True)
        hotwords = rospy.get_param("~hotword", [])
        try:
            self.hotwords = [ hotword.encode('utf-8') if isinstance(hotword, unicode ) else hotword
                              for hotword in hotwords ]
        except NameError:
            self.hotwords = hotwords

        self.state = State()
        self.queue = Queue.Queue()

        if self.use_tts:
            soundplay_action_name = rospy.get_param(
                '~soundplay_action_name', 'robotsound_jp')
            self.sound_action = actionlib.SimpleActionClient(
                soundplay_action_name, SoundRequestAction)
            if not self.sound_action.wait_for_server(rospy.Duration(5.0)):
                self.sound_action = None
            else:
                self.timer_speech = rospy.Timer(
                    rospy.Duration(0.1), self.speech_timer_cb)
        else:
            self.sound_action = None

        if self.use_audio:
            self.audio_config = df.types.InputAudioConfig(
                audio_encoding=df.enums.AudioEncoding.AUDIO_ENCODING_LINEAR_16,
                language_code=self.language,
                sample_rate_hertz=self.audio_sample_rate)
            self.audio_data = None
            self.sub_hotword = rospy.Subscriber(
                "hotword", String, self.hotword_cb)
            self.sub_audio = rospy.Subscriber(
                "speech_audio", AudioData, self.input_cb)
        else:
            self.sub_speech = rospy.Subscriber(
                "speech_to_text", SpeechRecognitionCandidates,
                self.input_cb)

        self.df_thread = threading.Thread(target=self.df_run)
        self.df_thread.daemon = True
        self.df_thread.start()

    def speech_timer_cb(self, event=None):
        if self.state != State.IDLE:
            if (rospy.Time.now() - self.state.last_changed
                    > rospy.Duration(self.timeout)):
                self.state.set(State.IDLE)
                self.session_id = None

    def hotword_cb(self, msg):
        if msg.data in self.hotwords:
            rospy.loginfo("Hotword received")
            self.state.set(State.LISTENING)

    def input_cb(self, msg):
        if not self.enable_hotword:
            self.state.set(State.LISTENING)
        elif not self.use_audio:
            # catch hotword from string
            if isinstance(msg, SpeechRecognitionCandidates):
                self.hotword_cb(String(data=msg.transcript[0]))
            else:
                rospy.logerr("Unsupported data class {}".format(msg))

        if self.state == State.LISTENING:
            self.queue.put(msg)
            rospy.loginfo("Received input")
        else:
            rospy.logdebug("Received input but ignored")

    def detect_intent_audio(self, data, session):
        query = df.types.QueryInput(audio_config=self.audio_config)
        return self.session_client.detect_intent(
            session=session, query_input=query,
            input_audio=data).query_result

    def print_result(self, result):
        rospy.loginfo(pprint.pformat(result))

    def publish_result(self, result):
        msg = self.make_dialog_msg(result)
        self.pub_res.publish(msg)

    def speak_result(self, result):
        if self.sound_action is None:
            return
        msg = SoundRequest(
            command=SoundRequest.PLAY_ONCE,
            sound=SoundRequest.SAY,
            volume=self.volume)

        # for japanese or utf-8 languages
        if self.language == 'ja-JP' and sys.version_info.major <= 2:
            msg.arg = result.fulfillment_text.encode('utf-8')
        else:
            msg.arg = result.fulfillment_text
        if self.language == 'ja-JP':
            msg.arg2 = self.language

        self.sound_action.send_goal_and_wait(
            SoundRequestGoal(sound_request=msg),
            rospy.Duration(10.0))

    def df_run(self):
        while True:
            if rospy.is_shutdown():
                break
            try:
                msg = self.queue.get(timeout=0.1)
                rospy.loginfo("Processing")
                if self.session_id is None:
                    self.session_id = str(uuid.uuid1())
                    rospy.loginfo(
                        "DialogflowAudioClient: Created new session: {}".format(self.session_id))
                session = self.session_client.session_path(
                    self.project_id, self.session_id)

                if isinstance(msg, AudioData):
                    result = self.detect_intent_audio(msg.data, session)
                elif isinstance(msg, SpeechRecognitionCandidates):
                    result = self.detect_intent_text(
                        msg.transcript[0], session)
                else:
                    raise RuntimeError("Invalid data")
                self.print_result(result)
                self.publish_result(result)
                self.speak_result(result)
                if result.action == "Bye":
                    self.state.set(State.IDLE)
                    self.session_id = None
            except Queue.Empty:
                pass
            except Exception as e:
                rospy.logerr(e)


if __name__ == '__main__':
    rospy.init_node("dialogflow_client")
    dftc = DialogflowTextClient()
    dfac = DialogflowAudioClient()
    rospy.spin()
