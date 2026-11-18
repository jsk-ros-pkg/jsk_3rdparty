import nfc
import nfc.tag
import nfc.tag.tt1
import nfc.tag.tt1_broadcom
import nfc.tag.tt2
import nfc.tag.tt2_nxp
import nfc.tag.tt3
import nfc.tag.tt3_sony
import nfc.tag.tt4

from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
from nfc_ros.msg import Tag
from nfc_ros.msg import NDEFRecord

from nfc_ros.nfc_interface import NFCInterface


def tag_constant(name):
    """Tag.<name>, which is upper cased on ROS2 (rosidl only allows upper
    case constant names, see CMakeLists.txt)"""
    return getattr(Tag, name, None) or getattr(Tag, name.upper())


class NFCROSNode:

    def __init__(self, node=None):
        self.node = node

        self.interface = NFCInterface()
        self.pub_tag = ros_compat.create_publisher(node, '~tag', Tag, queue_size=1)
        ros_compat.loginfo(node, 'Initialized')

    def publish_tag(self, tag):

        ros_compat.loginfo(self.node, 'tag: {}'.format(tag))
        msg = Tag()

        if type(tag.identifier) == bytes:
            msg.identifier = tag.identifier.hex()
        else:
            msg.identifier = tag.identifier

        if type(tag) == nfc.tag.tt1.Type1Tag:
            msg.tag_type = Tag.NFC_TAG_TYPE1TAG
        elif type(tag) == nfc.tag.tt2.Type2Tag:
            msg.tag_type = Tag.NFC_TAG_TYPE2TAG
        elif type(tag) == nfc.tag.tt2_nxp.NTAG215:
            msg.tag_type = Tag.NFC_TAG_TYPE2NXPNTAG215
        elif type(tag) == nfc.tag.tt3.Type3Tag:
            msg.tag_type = Tag.NFC_TAG_TYPE3TAG
        elif type(tag) == nfc.tag.tt3_sony.FelicaStandard:
            msg.tag_type = tag_constant('NFC_TAG_FelicaStandard')
        elif type(tag) == nfc.tag.tt3_sony.FelicaMobile:
            msg.tag_type = tag_constant('NFC_TAG_FelicaMobile')
        elif type(tag) == nfc.tag.tt3_sony.FelicaLite:
            msg.tag_type = tag_constant('NFC_TAG_FelicaLite')
        elif type(tag) == nfc.tag.tt3_sony.FelicaLiteS:
            msg.tag_type = tag_constant('NFC_TAG_FelicaLiteS')
        elif type(tag) == nfc.tag.tt3_sony.FelicaPlug:
            msg.tag_type = tag_constant('NFC_TAG_FelicaPlug')
        elif type(tag) == nfc.tag.tt4.Type4Tag:
            msg.tag_type = Tag.NFC_TAG_TYPE4TAG
        elif type(tag) == nfc.tag.tt4.Type4ATag:
            msg.tag_type = Tag.NFC_TAG_TYPE4ATAG
        elif type(tag) == nfc.tag.tt4.Type4BTag:
            msg.tag_type = Tag.NFC_TAG_TYPE4BTAG
        else:
            ros_compat.logerr(self.node, 'Unknown tag type: {}'.format(type(tag)))
            return True

        if tag.ndef is not None:
            for record in tag.ndef.records:
                ndef_record = NDEFRecord()
                ndef_record.type = record.type
                ndef_record.name = record.name
                ndef_record.data = record.data.decode()
                msg.ndef_records.append(ndef_record)

        self.pub_tag.publish(msg)
        return True

    def spin(self):

        while not ros_compat.is_shutdown(self.node):
            # return on shutdown, see __main__ of nfc_ros_node.py for ROS2
            self.interface.read_card(
                on_connect=self.publish_tag,
                terminate=lambda: ros_compat.is_shutdown(self.node))
