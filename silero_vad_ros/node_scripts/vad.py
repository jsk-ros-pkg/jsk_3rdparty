#!/usr/bin/env python

import struct

import numpy as np
import rospy
import torch

from webrtcvad_ros.vad_core import VADBaseNode


class SileroVADROS(VADBaseNode):

  def __init__(self):

    super(SileroVADROS, self).__init__()

    self.model_vad, _ = torch.hub.load(repo_or_dir='snakers4/silero-vad',
                                       model='silero_vad',
                                       force_reload=True)

  def _get_vad_confidence(self, chunk, sampling_rate):
    return self.model_vad(
        torch.from_numpy(self._convert_bytearray_to_numpy_array(chunk)),
        sampling_rate).item()

  def _convert_bytearray_to_numpy_array(self, data):
    if self._audio_info.sample_format == 'S16LE':
      return np.array(struct.unpack("{}h".format(len(data) / 2), data))
    else:
      return np.array([])


def main():

  rospy.init_node('_ros')
  node = WebRTCVADROS()
  rospy.spin()


if __name__ == '__main__':
  main()
