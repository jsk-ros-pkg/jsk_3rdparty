import nfc
import nfc.tag

import rospy
from nfc_ros.msg import Tag

from nfc_ros.nfc_interface import NFCInterface


class NFCROSNode:

    def __init__(self):

        self.interface = NFCInterface()
        self.pub_tag = rospy.Publisher('~tag', Tag, queue_size=1)

    def spin(self):

        while not rospy.is_shutdown():
            tag = self.interface.read_card()
            msg = Tag()
            msg.identifier = tag.identifier
            #if type(tag) == nfc.tag.tt3.Type3Tag:
            #    msg.tag_type = Tag.NFC_TAG_TYPE3TAG
            #elif type(tag) == nfc.tag.tt3_sony.FelicaStandard:
            #    msg.tag_type == Tag.NFC_TAG_FelicaStandard
            #elif type(tag) == nfc.tag.tt3_sony.FelicaMobile:
            #    msg.tag_type == Tag.NFC_TAG_FelicaMobile
            #elif type(tag) == nfc.tag.tt3_sony.FelicaLite:
            #    msg.tag_type == Tag.NFC_TAG_FelicaLite
            #elif type(tag) == nfc.tag.tt3_sony.FelicaLiteS:
            #    msg.tag_type == Tag.NFC_TAG_FelicaLiteS
            #elif type(tag) == nfc.tag.tt3_sony.FelicaPlug:
            #    msg.tag_type == Tag.NFC_TAG_FelicaPlug
            #elif type(tag) == nfc.tag.tt3.Type4Tag:
            #    msg.tag_type = Tag.NFC_TAG_TYPE4TAG
            #elif type(tag) == nfc.tag.tt3.Type4ATag:
            #    msg.tag_type = Tag.NFC_TAG_TYPE4ATAG
            #elif type(tag) == nfc.tag.tt3.Type4BTag:
            #    msg.tag_type = Tag.NFC_TAG_TYPE4BTAG
            #else:
            #    rospy.logerr('Unknown tag type: {}'.format(type(tag)))
            #    continue
            self.pub_tag.publish(msg)
