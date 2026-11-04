pio run
pio run -t uploadfs --upload-port /dev/ttyACM0
pio run -t upload --upload-port /dev/ttyACM0
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600