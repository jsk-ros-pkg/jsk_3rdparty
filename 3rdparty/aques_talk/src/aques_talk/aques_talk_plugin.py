# -*- coding: utf-8 -*-

import os
import tempfile

import rospkg

from sound_play.sound_play_plugin import SoundPlayPlugin


class AquesTalkPlugin(SoundPlayPlugin):
    _default_voice = 'aq_f1c'

    def __init__(self):
        super(AquesTalkPlugin, self).__init__()

    def sound_play_say_plugin(self, text, voice):
        if voice is None or voice == '':
            voice = self._default_voice
        txtfile = tempfile.NamedTemporaryFile(
            prefix='sound_play', suffix='.txt')
        (wavfile, wavfilename) = tempfile.mkstemp(
            prefix='sound_play', suffix='.wav')
        txtfilename = txtfile.name
        os.close(wavfile)

        try:
            try:
                if hasattr(text, 'decode'):
                    txtfile.write(
                        text.decode('UTF-8').encode('ISO-8859-15'))
                else:
                    txtfile.write(
                        text.encode('ISO-8859-15'))
            except UnicodeEncodeError:
                if hasattr(text, 'decode'):
                    txtfile.write(text)
                else:
                    txtfile.write(text.encode('UTF-8'))
            txtfile.flush()

            jptext_file = "/tmp/_voice_text_%s.txt" % os.getpid()
            phont_file = "{}.phont".format(voice)
            phont_file = "%s/phont/%s" % (
                rospkg.RosPack().get_path("aques_talk"), phont_file)

            # escape invalid code
            # For detail, please see README.md of aques_talk
            # sed command for Japanese
            # https://ja.stackoverflow.com/questions/29179/sed-%E3%81%A7-%E3%81%82-%E3%82%9E-%E3%81%AE%E3%82%88%E3%81%86%E3%81%AA%E6%97%A5%E6%9C%AC%E8%AA%9E%E3%81%AE%E6%96%87%E5%AD%97%E7%AF%84%E5%9B%B2%E3%82%92%E4%BD%BF%E3%81%84%E3%81%9F%E3%81%84
            os.system("unset LC_ALL && unset LC_CTYPE && export LC_COLLATE=C.UTF-8 && \
                       nkf -j %s | kakasi -JH | nkf -w | \
                       sed -e 's/，/、/g' | sed -e 's/,/、/g' | \
                       sed -e 's/．/。/g' | sed -e 's/\./。/g' | \
                       sed -e 's/！/。/g' | sed -e 's/\!/。/g' | \
                       sed -e 's/〜/ー/g' | \
                       sed -e 's/[^a-zA-Z0-9ぁ-んァ-ンー、。?？]//g' | \
                       sed -e 's/^[ぁぃぅぇぉァィゥェォ、。ー]*//' | \
                       sed -e 's/\([a-zA-Z]\+\)/<ALPHA VAL=\\1>/g' | \
                       sed -e 's/\([0-9]\+\)/<NUMK VAL=\\1>/g' > \
                       %s" % (txtfilename, jptext_file))
            os.system("cat %s 1>&2" % (jptext_file))
            os.system("rosrun aques_talk SampleTalk -p %s -o %s %s" % (
                phont_file, wavfilename, jptext_file))

            os.remove(jptext_file)
        finally:
            txtfile.close()
        return wavfilename
