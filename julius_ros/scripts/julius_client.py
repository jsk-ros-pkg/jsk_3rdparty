#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import actionlib
from threading import Lock
import os
import sys
import rospy
from sound_play.msg import SoundRequest, SoundRequestAction, SoundRequestGoal
from julius_ros.utils import make_phonemes_from_words
from julius_ros.utils import make_grammar_from_rules
from julius_ros.utils import make_voca_from_categories
from julius_ros.utils import make_dfa
from julius_ros.audio_transport import AudioTransport
from julius_ros.module_client import ModuleClient
from speech_recognition_msgs.msg import Grammar
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from speech_recognition_msgs.msg import Vocabulary
from speech_recognition_msgs.srv import SpeechRecognition
from speech_recognition_msgs.srv import SpeechRecognitionResponse
from std_srvs.srv import Empty, EmptyResponse
import lxml.etree


class JuliusClient(object):
    start_signal = "/usr/share/sounds/freedesktop/stereo/bell.ogg"
    success_signal = "/usr/share/sounds/freedesktop/stereo/message-new-instant.ogg"
    timeout_signal = "/usr/share/sounds/freedesktop/stereo/network-connectivity-lost.ogg"

    def __init__(self):
        # load parameters
        self.encoding = rospy.get_param("~encoding", "utf-8")
        self.default_duration = rospy.get_param("~duration", 3.0)
        self.default_threshold = rospy.get_param("~threshold", 0.8)
        self.use_isolated_word = rospy.get_param("~use_isolated_word", True)
        self.start_signal_action_timeout = rospy.get_param("~start_signal_action_timeout", 0.3)

        self.pub_speech_recognition = rospy.Publisher("speech_to_text",
                                                      SpeechRecognitionCandidates,
                                                      queue_size=1)

        # initialize sound play
        self.act_sound = actionlib.SimpleActionClient("sound_play", SoundRequestAction)
        if not self.act_sound.wait_for_server(rospy.Duration(5.0)):
            rospy.logwarn("Failed to find sound_play action. Disabled audio alert")
            self.act_sound = None

        # setup julius
        host = rospy.get_param("~host", "localhost")
        module_port = rospy.get_param("~module_port", 10500)
        audio_port = rospy.get_param("~audio_port", 10501)
        max_retry = rospy.get_param("~max_connection_retry", 0)

        self.module = ModuleClient(host, module_port, max_retry, self.encoding)
        self.audio = AudioTransport(host, audio_port, max_retry, "audio")

        rospy.on_shutdown(self.shutdown_cb)
        self.module.on_received_data(self.julius_cb)

        self.module.start()
        self.audio.start()

        self.status()
        self.start()

        # start subscribe
        self.lock = Lock()
        self.last_speech = SpeechRecognitionCandidates()
        self.vocabularies = {}

        self.srv_show_status = rospy.Service("show_julius_status",
                                             Empty, self.status)

        if self.use_isolated_word:
            self.sub_vocabulary = rospy.Subscriber("vocabulary", Vocabulary,
                                                   self.vocabulary_cb)
        else:
            self.sub_grammar = rospy.Subscriber("grammar", Grammar,
                                                self.grammar_cb)
        self.srv_speech_recognition = rospy.Service("speech_recognition",
                                                    SpeechRecognition,
                                                    self.speech_recognition_cb)

    def start(self):
        self.grammar_changed = None
        self.module.send_command(["INPUTONCHANGE", "PAUSE"])

    def status(self, args=None):
        self.module.send_command(["VERSION"])
        self.module.send_command(["STATUS"])
        self.module.send_command(["GRAMINFO"])
        return EmptyResponse()

    def play_sound(self, path, timeout=5.0):
        if self.act_sound is None:
            return
        req = SoundRequest()
        req.sound = SoundRequest.PLAY_FILE
        req.command = SoundRequest.PLAY_ONCE
        if hasattr(SoundRequest, 'volume'): # volume is added from 0.3.0 https://github.com/ros-drivers/audio_common/commit/da9623414f381642e52f59701c09928c72a54be7#diff-fe2d85580f1ccfed4e23a608df44a7f7
            req.volume = 1.0
        req.arg = path
        goal = SoundRequestGoal(sound_request=req)
        self.act_sound.send_goal_and_wait(goal, rospy.Duration(timeout))

    def send_grammar_cmd(self, cmd):
        self.grammar_changed = None
        self.module.send_command(cmd)
        while self.grammar_changed is None:
            rospy.sleep(0.01)
        return self.grammar_changed

    def activate_gram(self, name):
        with self.lock:
            cmd = ["ACTIVATEGRAM", name]
            self.module.send_command(cmd)
            ok = True
            if ok:
                rospy.loginfo("Successfully activated grammar")
            else:
                rospy.logerr("Failed to activate grammar")
            return ok

    def deactivate_gram(self, name):
        with self.lock:
            cmd = ["DEACTIVATEGRAM", name]
            rospy.loginfo("Deactivating %s" % name)
            self.module.send_command(cmd)
            ok = True
            if ok:
                rospy.loginfo("Successfully deactivated grammar")
            else:
                rospy.logerr("Failed to deactivate grammar")
            return ok

    def do_gram(self, cmd_name, name, dfa, dic):
        with self.lock:
            assert cmd_name
            assert name
            assert dic
            cmd = ["%s %s" % (cmd_name, name)]
            if dfa is not None:
                cmd += [' ' + l.strip() for l in dfa if l.strip()]
                cmd += ["DFAEND"]
            cmd += [' ' + l.strip() for l in dic if l.strip()]
            cmd += ["DICEND"]
            ok = self.send_grammar_cmd(cmd)
            if ok:
                rospy.loginfo("%s Success" % cmd_name)
            else:
                rospy.logerr("%s Failed" % cmd_name)
            return ok

    def add_gram(self, name, dfa, dic):
        ok = self.do_gram("ADDGRAM", name, dfa, dic)
        ok &= self.deactivate_gram(name)
        return ok

    def change_gram(self, name, dfa, dic):
        return self.do_gram("CHANGEGRAM", name, dfa, dic)

    def grammar_cb(self, msg):
        if msg.name:
            name = msg.name
        else:
            rospy.logwarn("Name of grammar is empty. Use 'unknown'")
            name = 'unknown'
        grammar = make_grammar_from_rules(msg.rules)
        voca = make_voca_from_categories(msg.categories, msg.vocabularies)
        result = make_dfa(grammar, voca)
        if result is None:
            rospy.logerr("Failed to make dfa from grammar message")
            return
        dfa, dic = result
        ok = self.add_gram(name, dfa.split(os.linesep), dic.split(os.linesep))
        if ok:
            self.vocabularies[name] = list(set(e for v in msg.vocabularies for e in v.words))
        else:
            rospy.logerr("Failed to change vocabulary")

    def vocabulary_cb(self, msg):
        if msg.name:
            name = msg.name
        else:
            rospy.logwarn("Name of grammar is empty. Use 'unknown'")
            name = 'unknown'
        if len(msg.phonemes) == 0 or not msg.phonemes[0]:
            phonemes = make_phonemes_from_words(msg.words)
        else:
            phonemes = msg.phonemes
        dic = [" %s\t%s" % (w, p) for w, p in zip(msg.words, phonemes)]
        ok = self.add_gram(name, None, dic)
        if ok:
            self.vocabularies[name] = list(set(msg.words))
        else:
            rospy.logerr("Failed to change vocabulary")

    def speech_recognition_cb(self, req):
        res = SpeechRecognitionResponse()

        # change grammar
        candidate_words = []
        if req.grammar_name:
            ok = self.activate_gram(req.grammar_name)
            if not ok:
                rospy.logerr("failed to activate grammar %s" % req.grammar_name)
                return res
            if req.grammar_name in self.vocabularies:
                candidate_words = self.vocabularies[req.grammar_name]
        elif req.grammar.rules:
            g = req.grammar
            if not g.name:
                g.name = 'unknown'
            grammar = make_grammar_from_rules(g.rules)
            voca = make_voca_from_categories(g.categories, g.vocabularies)
            result = make_dfa(grammar, voca)
            if result is None:
                rospy.logerr("Failed to make dfa from grammar message")
                return res
            dfa, dic = result
            ok = self.change_gram(g.name, dfa.split(os.linesep), dic.split(os.linesep))
            if not ok:
                rospy.logerr("Failed to change grammar")
                return res
            self.vocabularies[g.name] = list(set(e for v in msg.vocabularies for e in v.words))
            candidate_words = self.vocabularies[g.name]
        elif req.vocabulary.words:
            v = req.vocabulary
            if not v.name:
                v.name = 'unknown'
            if len(v.phonemes) == 0 or not v.phonemes[0]:
                v.phonemes = make_phonemes_from_words(v.words)
            dic = [" %s\t%s" % (w, p) for w, p in zip(v.words, v.phonemes)]
            ok = self.change_gram(v.name, None, dic)
            if not ok:
                rospy.logerr("Failed to change vocabulary")
                return res
            self.vocabularies[v.name] = list(set(v.words))
            candidate_words = self.vocabularies[v.name]
        else:
            rospy.logerr("Invalid request: 'grammar_name', 'grammar' or 'vocabulary' must be filled")
            return res

        duration = req.duration
        if duration <= 0.0:
            duration = self.default_duration

        threshold = req.threshold
        if threshold <= 0.0 or threshold > 1.0:
            threshold = self.default_threshold

        if not req.quiet:
            self.play_sound(self.start_signal, self.start_signal_action_timeout)
        start_time = rospy.Time.now()
        self.last_speech = SpeechRecognitionCandidates()
        while (rospy.Time.now() - start_time).to_sec() < duration:
            rospy.sleep(0.1)
            speech = self.last_speech
            if not self.last_speech.transcript:
                continue
            if candidate_words:
                ok = speech.transcript[0] in candidate_words and speech.confidence[0] >= threshold
            else:
                ok = speech.confidence[0] >= threshold
            if ok:
                t0 = speech.transcript[0]
                c0 = speech.confidence[0]
                rospy.loginfo("Recognized %s (%f)..." % (t0, c0))
                if not req.quiet:
                    self.play_sound(self.success_signal, 0.1)
                res.result = speech
                return res

        # timeout
        rospy.logerr("Timed out")
        if not req.quiet:
            self.play_sound(self.timeout_signal, 0.1)
        return res

    def process_result(self, data):
        results = {}
        for shypo in data.xpath("//SHYPO"):
            transcript = []
            confidence = 0.0
            for whypo in shypo.xpath("./WHYPO"):
                if sys.version_info.major < 3:
                    word = whypo.attrib["WORD"].encode(self.encoding)
                else:
                    word = whypo.attrib["WORD"]
                if word.startswith("<"):
                    continue
                transcript.append(word)
                confidence += float(whypo.attrib["CM"])
            confidence /= len(transcript)
            transcript = " ".join(transcript)
            results[confidence] = transcript

        msg = SpeechRecognitionCandidates()
        debug_str = ["Recognized:"]
        for i, result in enumerate(sorted(results.items(), reverse=True)):
            c, t = result
            debug_str += ["%d: %s (%.2f)" % (i+1, t, c)]
            msg.transcript.append(t)
            msg.confidence.append(c)
        self.pub_speech_recognition.publish(msg)
        self.last_speech = msg
        rospy.logdebug(os.linesep.join(debug_str))

    def shutdown_cb(self):
        self.module.join()
        self.audio.join()

    def julius_cb(self, data):
        status, detail = data
        rospy.logdebug("status: %s" % status)
        rospy.logdebug("detail: %s" % lxml.etree.tostring(detail))
        if status == 'ENGINEINFO':
            version = detail.attrib["VERSION"]
            conf = detail.attrib["CONF"]
            rospy.loginfo("Version: %s (%s)" % (version, conf))
        elif status == 'SYSINFO':
            rospy.loginfo("Status: %s" % detail.attrib["PROCESS"])
        elif status == 'GRAMINFO':
            rospy.loginfo("Grammar Information:\n%s" % detail.text.strip())
        elif status == 'STARTPROC':
            rospy.loginfo("Julius Engine initialized")
        elif status == 'ENDPROC':
            rospy.loginfo("Julius Engine stopped")
        elif status == 'STARTRECOG':
            rospy.logdebug("Start Recognize")
        elif status == 'ENDRECOG':
            rospy.logdebug("End Recognize")
        elif status == 'RECOGFAIL':
            rospy.logerr("Recognition Failed")
        elif status == 'RECOGOUT':
            self.process_result(detail)
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
            elif substat == "ERROR":
                reason = detail.attrib["REASON"]
                rospy.logerr("Failed to change grammar: %s" % reason)
                self.grammar_changed = False
        else:
            rospy.logwarn("Received %s" % status)
            rospy.logwarn("%s", lxml.etree.tostring(detail))

if __name__ == '__main__':
    rospy.init_node("julius_client")
    client = JuliusClient()
    rospy.spin()
