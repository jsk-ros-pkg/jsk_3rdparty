import nfc
import nfc.tag
import nfc.tag.tt1
import nfc.tag.tt1_broadcom
import nfc.tag.tt2
import nfc.tag.tt2_nxp
import nfc.tag.tt3
import nfc.tag.tt3_sony
import nfc.tag.tt4

import rospy
from nfc_ros.msg import Tag
from nfc_ros.msg import NDEFRecord

from nfc_ros.nfc_interface import NFCInterface


class NFCROSNode:

    def __init__(self):

        self.interface = NFCInterface()
        self.pub_tag = rospy.Publisher('~tag', Tag, queue_size=1)
        rospy.loginfo('Initialized')

    def publish_tag(self, tag):

        rospy.loginfo('tag: {}'.format(tag))
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
            msg.tag_type == Tag.NFC_TAG_FelicaStandard
        elif type(tag) == nfc.tag.tt3_sony.FelicaMobile:
            msg.tag_type == Tag.NFC_TAG_FelicaMobile
        elif type(tag) == nfc.tag.tt3_sony.FelicaLite:
            msg.tag_type == Tag.NFC_TAG_FelicaLite
        elif type(tag) == nfc.tag.tt3_sony.FelicaLiteS:
            msg.tag_type == Tag.NFC_TAG_FelicaLiteS
        elif type(tag) == nfc.tag.tt3_sony.FelicaPlug:
            msg.tag_type == Tag.NFC_TAG_FelicaPlug
        elif type(tag) == nfc.tag.tt4.Type4Tag:
            msg.tag_type = Tag.NFC_TAG_TYPE4TAG
        elif type(tag) == nfc.tag.tt4.Type4ATag:
            msg.tag_type = Tag.NFC_TAG_TYPE4ATAG
        elif type(tag) == nfc.tag.tt4.Type4BTag:
            msg.tag_type = Tag.NFC_TAG_TYPE4BTAG
        else:
            rospy.logerr('Unknown tag type: {}'.format(type(tag)))
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

        while not rospy.is_shutdown():
            self.interface.read_card(on_connect=self.publish_tag)
