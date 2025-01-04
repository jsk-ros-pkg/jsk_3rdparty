import struct

from smart_device_protocol.msg import Packet


# Version 1 of the packet generator
def create_test_packet(
    num_int: int = -120, num_float: float = -1.0, string: str = "Hello, world!"
):
    return (
        struct.pack("<H", Packet.PACKET_TYPE_TEST)
        + struct.pack("<i", num_int)
        + struct.pack("<f", num_float)
        + struct.pack("64s", string.encode("utf-8"))
    )


def create_task_dispatcher_packet(
    caller_name: str, target_name: str, task_name: str, task_args: str = ""
):
    return (
        struct.pack("<H", Packet.PACKET_TYPE_TASK_DISPATCHER)
        + struct.pack("16s", caller_name.encode("utf-8"))
        + struct.pack("16s", target_name.encode("utf-8"))
        + struct.pack("16s", task_name.encode("utf-8"))
        + struct.pack("{}s".format(max(1, len(task_args))), task_args.encode("utf-8"))
    )


def create_task_received_packet(worker_name: str, caller_name: str, task_name: str):
    return (
        struct.pack("<H", Packet.PACKET_TYPE_TASK_RECEIVED)
        + struct.pack("16s", worker_name.encode("utf-8"))
        + struct.pack("16s", caller_name.encode("utf-8"))
        + struct.pack("16s", task_name.encode("utf-8"))
    )


def create_emergency_packet(
    map_frame: str,
    position_x: float,
    position_y: float,
    position_z: float,
    orientation_x: float,
    orientation_y: float,
    orientation_z: float,
    orientation_w: float,
):
    return (
        struct.pack("<H", Packet.PACKET_TYPE_EMERGENCY)
        + struct.pack("64s", map_frame.encode("utf-8"))
        + struct.pack("<f", position_x)
        + struct.pack("<f", position_y)
        + struct.pack("<f", position_z)
        + struct.pack("<f", orientation_x)
        + struct.pack("<f", orientation_y)
        + struct.pack("<f", orientation_z)
        + struct.pack("<f", orientation_w)
    )


def create_device_message_board_data(
    source_name: str, timeout_duration: int, message: str
):
    return (
        struct.pack("<H", Packet.PACKET_TYPE_DEVICE_MESSAGE_BOARD_DATA)
        + struct.pack("64s", source_name.encode("utf-8"))
        + struct.pack("<Q", int(timeout_duration))
        + struct.pack("64s", message.encode("utf-8"))
    )
