import struct

import rospy

from smart_device_protocol.msg import Packet


class ESPNOWROSInterface:
    def __init__(
        self,
        callback=None,
        recv_topic="/smart_device_protocol/recv",
        send_topic="/smart_device_protocol/send",
    ):
        self.raw_callback = callback
        self.sub = rospy.Subscriber(recv_topic, Packet, self.callback)
        self.pub = rospy.Publisher(send_topic, Packet, queue_size=1)

    def callback(self, msg):
        src_address = struct.unpack("6B", msg.mac_address)
        data = msg.data
        if self.raw_callback is not None:
            self.raw_callback(src_address, data)

    def send(self, target_address, data, num_trial=1):
        """
        Args:
            target_address (list of int)
            data (bytes)
        """
        msg = Packet()
        msg.mac_address = struct.pack(
            "6B",
            target_address[0],
            target_address[1],
            target_address[2],
            target_address[3],
            target_address[4],
            target_address[5],
        )
        msg.data = data
        for _ in range(num_trial):
            self.pub.publish(msg)
