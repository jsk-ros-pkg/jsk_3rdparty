#!/usr/bin/env python

import struct

import numpy as np
import rospy
import torch

from webrtcvad_ros.vad_core import VADBaseNode


class SileroVADROS(VADBaseNode):

    def __init__(self):

        model_vad, _ = torch.hub.load(
                repo_or_dir="snakers4/silero-vad:v5.0", model="silero_vad", force_reload=True
        )
        self.model_vad = model_vad

        super(SileroVADROS, self).__init__(chunk_size=512)

        rospy.loginfo("Initialized.")

    def _get_vad_confidence(self, chunk, sampling_rate):
        audio_chunk = np.frombuffer(chunk, np.int16)
        abs_max = np.abs(audio_chunk).max()
        audio_chunk = audio_chunk.astype("float32")
        if abs_max > 0:
            audio_chunk *= 1 / 32768
        audio_chunk = audio_chunk.squeeze()
        return self.model_vad(torch.from_numpy(audio_chunk), sampling_rate).item()


def main():

    rospy.init_node("silero_vad_ros")
    node = SileroVADROS()
    rospy.spin()


if __name__ == "__main__":
    main()
