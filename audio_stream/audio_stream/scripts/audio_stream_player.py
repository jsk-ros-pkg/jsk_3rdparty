#!/usr/bin/env python
# -*- coding:utf-8 -*-

import pyaudio
import rospy
import Queue

class AudioStreamPlayer:

    def __init__(self):
        #
        rospy.init_node( 'audio_stream_player' )
        # parameters
        loop_rate = rospy.get_param( '~loop_rate', 100 )
        use_audio_common = rospy.get_param( '~use_audio_common', False )
        topicname_data = '~audio'
        topicname_info = '~info'
        # buffer
        self.buffer = Queue.Queue()
        self.rate = rospy.Rate(loop_rate)
        # import message modules
        if use_audio_common:
            from audio_common_msgs.msg import AudioData
            from audio_stream_msgs.msg import AudioInfo
        else:
            from audio_stream_msgs.msg import AudioData, AudioInfo
        # get an info message
        info = rospy.wait_for_message( topicname_info, AudioInfo )
        audio_stream_channels = info.channels
        audio_stream_sampling_rate = info.sampling_rate
        audio_stream_width = info.width
        # start subscriber
        self.subscriber = rospy.Subscriber(topicname_data,AudioData,self.cbROS) 
        # setting up pyaudio
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open( 
                              format=self.p.get_format_from_width(info.width),
                              channels=info.channels,
                              rate=info.sampling_rate,
                              output=True )

    def cbROS(self,msg):
        self.buffer.put(msg.data)

    def initialBuffering(self,duration=5.0):
        rospy.sleep(duration)
        num = self.buffer.qsize()
        for i in range(num):
            self.stream.write(self.buffer.get())

    def spin(self):
        self.rate.sleep()
        self.stream.start_stream()
        while not rospy.is_shutdown():
            if not self.stream.is_active():
                break
            self.rate.sleep()
            num = self.buffer.qsize()
            for i in range(num):
                self.stream.write(self.buffer.get())
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()

def main():
    a = AudioStreamPlayer()
    a.initialBuffering(1.0)
    a.spin()

if __name__=='__main__':
    main()
