# M5AtomS3 Sesami Client

This sketch is for sesami client with M5AtomS3.

## How to use

First, update parameters in the skectch below

- SSID
- Password
- device UUID
- secret key
- api key

Then, burn the sketch to M5AtomS3

## Serial Interface

This device can be controlled with serial command via GROVE port.

example is

```
{"command": "toggle"}\n
```

```
{"command": "lock"}\n
```

```
{"command": "unlock"}\n
```

```
{"command": "status"}\n
```

```
{"command": "history"}\n
```

And result will be sent via serial port like

```
{"success":true,"message":"toggle success"}\n
```

## Notice

See the pages below if you have a problem with WiFiClientSecure.

- https://www.mgo-tec.com/blog-entry-arduino-esp32-wificlientsecure-hangup-solution.html
- https://www.1ft-seabass.jp/memo/2017/07/31/esp32-developer-aws-iot-tips/
