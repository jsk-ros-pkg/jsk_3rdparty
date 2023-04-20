#!/usr/bin/env python

import struct

import numpy as np
import rospy
import torch

from webrtcvad_ros.vad_core import VADBaseNode


class SileroVADROS(VADBaseNode):

  def __init__(self):

    model_vad, _ = torch.hub.load(repo_or_dir='snakers4/silero-vad',
                                  model='silero_vad',
                                  force_reload=True)
    self.model_vad = model_vad

    super(SileroVADROS, self).__init__()

    rospy.loginfo('Initialized.')

  def _get_vad_confidence(self, chunk, sampling_rate):
    print('chunk: {}'.format(chunk))
    nparray = self._convert_bytearray_to_numpy_array(chunk)
    print('nparray: {}'.format(nparray))
    return self.model_vad(torch.from_numpy(nparray), sampling_rate).item()

  def _convert_bytearray_to_numpy_array(self, data):
    if self._audio_info.sample_format == 'S16LE':
      return np.array(struct.unpack("{}h".format(int(len(data) / 2)), data))
    else:
      raise ValueError()


def main():

  rospy.init_node('silero_vad_ros')
  node = SileroVADROS()
  rospy.spin()


if __name__ == '__main__':
  main()
