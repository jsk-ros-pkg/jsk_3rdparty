import struct
from typing import Callable, Dict, List, Optional, Tuple, Union

import rospy

from smart_device_protocol.esp_now_ros_interface import ESPNOWROSInterface
from smart_device_protocol.msg import Packet
from smart_device_protocol.packet_parser import parse_packet_as_v2
from smart_device_protocol.sdp_frames import BaseFrame, DataFrame, MetaFrame


class SDPInterface:
    """Smart Device Protocol Interface"""

    def __init__(
        self,
        callback_data: Optional[
            Callable[[Union[List[int], Tuple[int]], BaseFrame], None]
        ] = None,
        callback_meta: Optional[
            Callable[[Union[List[int], Tuple[int]], BaseFrame], None]
        ] = None,
    ):
        """Smart Device Protocol Interface

        Args:
            callback_data (Optional[ Callable[[Union[List[int], Tuple[int]], BaseFrame], None] ], optional): callback function for DataFrame. Defaults to None.
            callback_meta (Optional[ Callable[[Union[List[int], Tuple[int]], BaseFrame], None] ], optional): callback function for MetaFrame. Defaults to None.
        """
        self._callback_data = callback_data
        self._callback_meta = callback_meta
        self._smart_device_protocol_interface = ESPNOWROSInterface(self._callback)

    def _callback(self, src_address, data):
        try:
            _, frame = parse_packet_as_v2(
                Packet(data=data, mac_address=struct.pack("6B", *list(src_address)))
            )
        except ValueError:
            return
        if isinstance(frame, DataFrame) and self._callback_data is not None:
            self._callback_data(src_address, frame)
        elif isinstance(frame, MetaFrame) and self._callback_meta is not None:
            self._callback_meta(src_address, frame)

    def send(self, target_address, frame, num_trial=1):
        if isinstance(frame, DataFrame):
            data = frame.to_bytes()
        elif isinstance(frame, MetaFrame):
            data = frame.to_bytes()
        else:
            raise ValueError(f"Unknown frame type: {type(frame)}")
        self._smart_device_protocol_interface.send(target_address, data, num_trial)


class DeviceDictSDPInterface(SDPInterface):
    """Smart Device Protocol Interface with Device Dictionary"""

    def __init__(self, callback_data=None, callback_meta=None, timeout: float = 10.0):
        self._device_interfaces: Dict[Union[List[int], Tuple[int]], Dict] = {}
        self._original_callback_data = callback_data
        self._original_callback_meta = callback_meta
        self._timeout = rospy.Duration(timeout)
        super().__init__(
            self._original_callback_data, self._callback_meta_for_device_interfaces
        )

    def _callback_meta_for_device_interfaces(self, src_address, frame):
        self._remove_timeout_device()
        self._device_interface_meta_callback(src_address, frame)
        if self._original_callback_meta is not None:
            self._original_callback_meta(src_address, frame)

    def _device_interface_meta_callback(self, src_address, frame):
        if src_address not in self._device_interfaces:
            self._device_interfaces[src_address] = {}
            self._device_interfaces[src_address]["device_name"] = frame.device_name
            self._device_interfaces[src_address]["interfaces"] = []

        self._device_interfaces[src_address]["last_stamp"] = rospy.Time.now()

        for interface_description in frame.interface_descriptions:
            if (interface_description != ("", "")) and (
                interface_description
                not in self._device_interfaces[src_address]["interfaces"]
            ):
                self._device_interfaces[src_address]["interfaces"].append(
                    interface_description
                )

    def _remove_timeout_device(self):
        now = rospy.Time.now()
        for src_address in list(self._device_interfaces.keys()):
            device_interface = self._device_interfaces[src_address]
            if now - device_interface["last_stamp"] > self._timeout:
                rospy.logwarn(
                    "Remove timeout device: {}, {}".format(
                        src_address, device_interface["device_name"]
                    )
                )
                self._device_interfaces.pop(src_address)

    @property
    def device_interfaces(self):
        return self._device_interfaces

    def send(
        self, target: Union[Tuple, str], frame: Union[DataFrame, MetaFrame], num_trial=1
    ):
        if isinstance(target, str):
            for src_address, device_interface in self._device_interfaces.items():
                if device_interface["device_name"] == target:
                    target_address = src_address
                    break
            else:
                raise ValueError(f"Unknown device name: {target}")
        else:
            target_address = target
        super().send(target_address, frame, num_trial)


class DeviceDictSDPInterfaceWithInterfaceCallback(DeviceDictSDPInterface):
    def __init__(
        self,
        callbacks_data: Dict[
            Tuple[str, str], Callable[[Union[List[int], Tuple[int]], List], None]
        ] = {},
        callback_meta=None,
        timeout: float = 10.0,
    ):
        self._callbacks_data = callbacks_data
        self._callback_meta = callback_meta
        super().__init__(
            self._callback_data_for_interface, self._callback_meta, timeout
        )

    def _callback_data_for_interface(self, src_address, frame):
        interface_description = frame.interface_description
        if interface_description in self._callbacks_data:
            self._callbacks_data[interface_description](src_address, frame)

    def register_callback(self, interface_description, callback):
        self._callbacks_data[interface_description] = callback

    def unregister_callback(self, interface_description):
        del self._callbacks_data[interface_description]
