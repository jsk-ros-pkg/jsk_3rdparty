import struct
from typing import List, Tuple, Union

from smart_device_protocol.sdp_frames import MetaFrame, DataFrame
from smart_device_protocol.msg import Packet

PACKET_TYPE_META = Packet.PACKET_TYPE_META
PACKET_TYPE_DATA = Packet.PACKET_TYPE_DATA


def parse_packet_as_v2(packet: Packet) -> Tuple[Tuple, Union[MetaFrame, DataFrame]]:
    src_address = struct.unpack("6B", packet.mac_address)
    packet_type = struct.unpack("<H", packet.data[0:2])[0]
    if packet_type == PACKET_TYPE_META:
        return src_address, MetaFrame.from_bytes(packet.data)
    elif packet_type == PACKET_TYPE_DATA:
        return src_address, DataFrame.from_bytes(packet.data)
    else:
        raise ValueError(f"Unknown packet type: {packet_type}")


# Version 1 of the packet parser
def parse_packet(packet):
    packet_type = struct.unpack("<H", packet[0:2])[0]

    if packet_type == Packet.PACKET_TYPE_NONE:
        return Packet.PACKET_TYPE_NONE, None

    elif packet_type == Packet.PACKET_TYPE_TEST:
        number_int = struct.unpack("<i", packet[2:6])[0]
        number_float = struct.unpack("<f", packet[6:10])[0]
        string = (
            struct.unpack("64s", packet[10:74])[0].decode("utf-8").replace("\x00", "")
        )
        return packet_type, number_int, number_float, string

    elif packet_type == Packet.PACKET_TYPE_SENSOR_ENV_III:
        module_name = (
            struct.unpack("64s", packet[2:66])[0].decode("utf-8").replace("\x00", "")
        )
        pressure = struct.unpack("<i", packet[66:70])[0]
        return packet_type, module_name, pressure

    elif packet_type == Packet.PACKET_TYPE_SENSOR_UNITV2_PERSON_COUNTER:
        number_of_person = struct.unpack("<i", packet[2:6])[0]
        place_name = (
            struct.unpack("64s", packet[6:70])[0].decode("utf-8").replace("\x00", "")
        )
        return packet_type, number_of_person, place_name

    elif packet_type == Packet.PACKET_TYPE_TASK_DISPATCHER:
        caller_name = (
            struct.unpack("16s", packet[2:18])[0].decode("utf-8").replace("\x00", "")
        )
        target_name = (
            struct.unpack("16s", packet[18:34])[0].decode("utf-8").replace("\x00", "")
        )
        task_name = (
            struct.unpack("16s", packet[34:50])[0].decode("utf-8").replace("\x00", "")
        )
        if len(packet) > 50:
            task_args = (
                struct.unpack("{}s".format(len(packet) - 50), packet[50:])[0]
                .decode("utf-8")
                .replace("\x00", "")
            )
        else:
            task_args = ""
        return packet_type, caller_name, target_name, task_name, task_args

    elif packet_type == Packet.PACKET_TYPE_TASK_RECEIVED:
        worker_name = (
            struct.unpack("16s", packet[2:18])[0].decode("utf-8").replace("\x00", "")
        )
        caller_name = (
            struct.unpack("16s", packet[18:34])[0].decode("utf-8").replace("\x00", "")
        )
        task_name = (
            struct.unpack("16s", packet[34:50])[0].decode("utf-8").replace("\x00", "")
        )
        return packet_type, worker_name, caller_name, task_name

    elif packet_type == Packet.PACKET_TYPE_TASK_RESULT:
        caller_name = (
            struct.unpack("16s", packet[2:18])[0].decode("utf-8").replace("\x00", "")
        )
        target_name = (
            struct.unpack("16s", packet[18:34])[0].decode("utf-8").replace("\x00", "")
        )
        task_name = (
            struct.unpack("16s", packet[34:50])[0].decode("utf-8").replace("\x00", "")
        )
        if len(packet) > 50:
            task_result = (
                struct.unpack("{}s".format(len(packet) - 50), packet[50:])[0]
                .decode("utf-8")
                .replace("\x00", "")
            )
        else:
            task_result = ""
        return packet_type, caller_name, target_name, task_name, task_result

    elif packet_type == Packet.PACKET_TYPE_DEVICE_MESSAGE_BOARD_META:
        device_name = (
            struct.unpack("64s", packet[2 : 2 + 64])[0]
            .decode("utf-8")
            .replace("\x00", "")
        )
        return packet_type, device_name

    elif packet_type == Packet.PACKET_TYPE_DEVICE_MESSAGE_BOARD_DATA:
        source_name = (
            struct.unpack("64s", packet[2 : 2 + 64])[0]
            .decode("utf-8")
            .replace("\x00", "")
        )
        timeout_duration = struct.unpack("<Q", packet[2 + 64 : 2 + 64 + 8])[0]
        message = (
            struct.unpack("64s", packet[2 + 64 + 8 : 2 + 64 + 8 + 64])[0]
            .decode("utf-8")
            .replace("\x00", "")
        )
        return packet_type, source_name, timeout_duration, message

    else:
        print("{} is not supported packet type", format(packet_type))
        return Packet.PACKET_TYPE_NONE, None
