#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import actionlib
import rospy
from sound_play.msg import SoundRequest, SoundRequestAction, SoundRequestGoal
from julius_ros.utils import make_phonemes_from_words
from julius_ros.audio_transport import AudioTransport
from julius_ros.module_client import ModuleClient
from speech_recognition_msgs.msg import SpeechRecognitionCandidates, Vocabulary
from speech_recognition_msgs.srv import SpeechRecognition, SpeechRecognitionResponse
import lxml.etree


class JuliusClient(object):
    start_signal = "/usr/share/sounds/ubuntu/stereo/bell.ogg"
    success_signal = "/usr/share/sounds/ubuntu/stereo/message-new-instant.ogg"
    timeout_signal = "/usr/share/sounds/ubuntu/stereo/window-slide.ogg"

    def __init__(self):
        host = rospy.get_param("~host", "localhost")
        module_port = rospy.get_param("~module_port", 10500)
        audio_port = rospy.get_param("~audio_port", 10501)
        max_retry = rospy.get_param("~max_connection_retry", 0)

        self.encoding = rospy.get_param("~encoding", "utf-8")
        self.default_duration = rospy.get_param("~duration", 3.0)
        self.default_threshold = rospy.get_param("~threshold", 0.8)

        self.pub_speech_recognition = rospy.Publisher("speech_to_text",
                                                      SpeechRecognitionCandidates,
                                                      queue_size=1)
        self.act_sound = actionlib.SimpleActionClient("sound_play", SoundRequestAction)
        if not self.act_sound.wait_for_server(rospy.Duration(5.0)):
            rospy.logwarn("Failed to find sound_play action. Disabled audio alert")
            self.act_sound = None

        self.module = ModuleClient(host, module_port, max_retry, self.encoding)
        self.audio = AudioTransport(host, audio_port, max_retry, "audio")

        rospy.on_shutdown(self.shutdown_cb)
        self.module.on_received_data(self.julius_cb)

        self.module.start()
        self.audio.start()

        self.status()
        self.start()

        self.last_speech = SpeechRecognitionCandidates()
        self.sub_vocabulary = rospy.Subscriber("vocabulary",
                                               Vocabulary,
                                               self.vocabulary_cb)
        self.srv_speech_recognition = rospy.Service("speech_recognition",
                                                    SpeechRecognition,
                                                    self.speech_recognition_cb)

    def start(self):
        self.grammar_changed = None
        self.module.send_command(["INPUTONCHANGE PAUSE"])

    def status(self):
        self.module.send_command(["VERSION"])
        self.module.send_command(["STATUS"])
        self.module.send_command(["GRAMINFO"])

    def play_sound(self, path, timeout=5.0):
        if self.act_sound is None:
            return
        req = SoundRequest()
        req.sound = SoundRequest.PLAY_FILE
        req.command = SoundRequest.PLAY_ONCE
        req.arg = path
        goal = SoundRequestGoal(sound_request=req)
        self.act_sound.send_goal_and_wait(goal, rospy.Duration(timeout))

    def change_gram(self, words, phonemes=[]):
        if len(phonemes) == 0 or not phonemes[0]:
            phonemes = make_phonemes_from_words(words)
        else:
            phonemes = phonemes

        # change grammar
        dic = [" %s %s" % (w, p) for w, p in zip(words, phonemes)]
        cmd = ["CHANGEGRAM julius_ros"] + dic + ["DICEND"]
        self.grammar_changed = None
        self.module.send_command(cmd)
        while self.grammar_changed is None:
            rospy.sleep(0.01)
        return self.grammar_changed

    def vocabulary_cb(self, msg):
        ok = self.change_gram(msg.words, msg.phonemes)
        if not ok:
            rospy.logerr("Failed to change vocabulary")

    def speech_recognition_cb(self, req):
        res = SpeechRecognitionResponse()
        voca = req.vocabulary
        if not voca.words:
            rospy.logerr("No words specified")
            return res

        ok = self.change_gram(voca.words, voca.phonemes)
        if not ok:
            rospy.logerr("Failed to change vocabulary")
            return res

        duration = req.duration
        if duration <= 0.0:
            duration = self.default_duration

        threshold = req.threshold
        if threshold <= 0.0 or threshold > 1.0:
            threshold = self.default_threshold

        if not req.quiet:
            self.play_sound(self.start_signal)
        start_time = rospy.Time.now()
        self.last_speech = SpeechRecognitionCandidates()
        while (rospy.Time.now() - start_time).to_sec() < duration:
            rospy.sleep(0.1)
            speech = self.last_speech
            if not self.last_speech.transcript:
                continue
            if speech.transcript[0] in voca.words and speech.confidence[0] >= threshold:
                rospy.loginfo("Recognized %s (%f)..." % (speech.transcript[0], speech.confidence[0]))
                if not req.quiet:
                    self.play_sound(self.success_signal, 0.1)
                res.results = speech
                return res

        # timeout
        rospy.logerr("Timed out")
        if not req.quiet:
            self.play_sound(self.timeout_signal, 0.1)
        return res

    def shutdown_cb(self):
        self.module.join()
        self.audio.join()

    def julius_cb(self, data):
        status, detail = data
        if status == 'ENGINEINFO':
            version = detail.attrib["VERSION"]
            conf = detail.attrib["CONF"]
            rospy.loginfo("Version: %s (%s)" % (version, conf))
        elif status == 'SYSINFO':
            rospy.loginfo("Status: %s" % detail.attrib["PROCESS"])
        elif status == 'GRAMINFO':
            rospy.logdebug("Grammar Information:\n%s" % detail.text.strip())
        elif status == 'STARTPROC':
            rospy.loginfo("Julius Engine initialized")
        elif status == 'ENDPROC':
            rospy.loginfo("Julius Engine stopped")
        elif status == 'STARTRECOG':
            rospy.logdebug("Start Recognize")
        elif status == 'ENDRECOG':
            rospy.logdebug("End Recognize")
        elif status == 'RECOGFAIL':
            rospy.logerr("Bad Recognize")
        elif status == 'RECOGOUT':
            whypo = detail.xpath('//*[local-name() = "WHYPO"]')
            msg = SpeechRecognitionCandidates()
            msg.transcript = [e.attrib["WORD"].encode(self.encoding) for e in whypo]
            msg.confidence = [float(e.attrib["CM"]) for e in whypo]
            self.last_speech = msg
            self.pub_speech_recognition.publish(msg)
        elif status == 'INPUT':
            substat = detail.attrib["STATUS"]
            if substat == 'STARTREC':
                rospy.logdebug("Detect speak start")
            elif substat == 'ENDREC':
                rospy.logdebug("Detect speak end")
        elif status == 'INPUTPARAM':
            input_frame = int(detail.attrib["FRAMES"])
            if input_frame >= 2000:
                rospy.logwarn("Audio segment is too long!! Please volume down your microphone.")
        elif status == 'GRAMMAR':
            substat = detail.attrib["STATUS"]
            if substat == 'RECEIVED':
                self.grammar_changed = True
            else:
                self.grammar_changed = False
        else:
            rospy.logdebug("Received %s" % status)
            rospy.logdebug("%s", lxml.etree.tostring(detail))

if __name__ == '__main__':
    rospy.init_node("julius_client")
    client = JuliusClient()
    rospy.spin()
