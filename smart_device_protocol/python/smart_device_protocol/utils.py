import importlib
from typing import Tuple


def import_ros_type(ros_type_str: str) -> type:
    package_name, message_name = ros_type_str.split("/")
    module = importlib.import_module(f"{package_name}.msg")
    rostype = getattr(module, message_name)
    return rostype


def address_tuple_to_str(address: Tuple) -> str:
    return "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}".format(
        address[0],
        address[1],
        address[2],
        address[3],
        address[4],
        address[5],
    )


def address_str_to_tuple(address: str) -> Tuple:
    return tuple([int(x, 16) for x in address.split(":")])
