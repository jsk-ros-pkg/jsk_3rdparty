# M5AtomS3 Switchbot Client

This sketch is for switchbot client with M5AtomS3.

## How to use

First, update parameters in the skectch below

- SSID
- Password
- token
- secret

Then, burn the sketch to M5AtomS3

## Serial Interface

This device can be controlled with serial command via GROVE port.

example is

```
{"command": "get_device_list"}\n
```

```
{"command": "get_device_status", "device_id": "<device ID>"}\n
```

```
{"command": "send_device_command", "device_id": "<device ID>", "pb_command_type": "<commandType>", "pb_command": "<command>"}\n
```

And result will be sent via serial port like

```
{"success":true,"message":"get_device_list success"}\n
```