#!/usr/bin/env python

import rospy
import webrtcvad

from webrtcvad_ros.vad_core import VADBaseNode


class WebRTCVADROS(VADBaseNode):

  def __init__(self):

    aggressiveness = rospy.get_param('~aggressiveness', 1)
    self._vad = webrtcvad.Vad(int(aggressiveness))

    super(WebRTCVADROS, self).__init__(chunk_size=480)

  def _get_vad_confidence(self, chunk, sampling_rate):
    return 1.0 if self._vad.is_speech(chunk, sampling_rate) else 0.0


def main():

  rospy.init_node('webrtcvad_ros')
  node = WebRTCVADROS()
  rospy.spin()


if __name__ == '__main__':
  main()
