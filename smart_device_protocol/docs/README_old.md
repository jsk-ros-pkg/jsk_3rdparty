# old readme for SMART_DEVICE_PROTOCOL

## Packet Specification

### System packet

#### Test Packet

| PACKET_TYPE (2 byte unsigned int LSB) | Integer number = -120 (4 byte signed int LSB) | Float number -1.0 (4 byte float LSB) | String (Hello, world!) (64 bytes String) |
|-|-|-|-|

### Named value packet

#### Named String Packet

| PACKET_TYPE (2 byte unsigned int LSB) | Name (64 byte string) | Value (64 bytes String) |
|-|-|-|

#### Named Int Packet

| PACKET_TYPE (2 byte unsigned int LSB) | Name (64 byte string) | Value (4 bytes signed int LSB) |
|-|-|-|

#### Named Float Packet

| PACKET_TYPE (2 byte unsigned int LSB) | Name (64 byte string) | Value (4 bytes float LSB) |
|-|-|-|

### Sensor modules

#### ENV III Packet

| PACKET_TYPE (2 byte unsigned int LSB) | MODULE_NAME (64 byte String) | PRESSURE (4 byte signed int LSB) |
|-|-|-|

#### UNITV2 Person Counter Packet

| PACKET_TYPE (2 byte unsigned int LSB) | NUMBER OF PERSON (4 byte unsigned int LSB) | PLACE_NAME (64 byte String) |
|-|-|-|

#### IMU Packet

| PACKET_TYPE (2 byte unsigned int LSB) | MODULE_NAME (64 byte String) | Accel_X, Y, Z (4 byte float LSB x 3) |
|-|-|-|

### Task packet

#### Emergency Packet

| PACKET_TYPE (2 byte unsigned int LSB) | MAP_FRAME (64 byte String) | POS_X, Y, Z (4 byte float LSB x 3) | ROT_X, Y, Z, W (4byte float LSB x 4) |
|-|-|-|-|

#### Task Dispatcher Packet

| PACKET_TYPE (2 byte unsigned int LSB) | Caller name (16 byte String) | Target name (16 byte String) | Task name (16 byte String) | Task Args (String) |
|-|-|-|-|-|

#### Task Received Packet

| PACKET_TYPE (2 byte unsigned int LSB) | Worker name (16 byte String) | Caller name (16 byte String) | Task name (16 byte String) |
|-|-|-|-|

#### Task Result Packet

| PACKET_TYPE (2 byte unsigned int LSB) | Worker name (16 byte String) | Caller name (16 byte String) | Task name (16 byte String) | Result (String) |
|-|-|-|-|-|

### Device Packet

#### Device message board meta packet

| PACKET_TYPE (2 byte unsigned int LSB) | device name (64 byte String) |
|-|-|

#### Device message board data packet

| PACKET_TYPE (2 byte unsigned int LSB) | source name (64 byte String) | timeout duration (msec) (8 byte unsigned int LSB) | message name (64 byte String) |
|-|-|-|-|
