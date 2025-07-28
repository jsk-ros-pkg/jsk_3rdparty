#!/usr/bin/env python

I2C_SLAVE = 0x0703

from i2c_for_esp32 import WirePacker  # pip3 install i2c-for-esp32

import sys
import io
import fcntl
import time

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point

if sys.hexversion < 0x03000000:
   def _b(x):
      return x
else:
   def _b(x):
      return x.encode('latin-1')

class I2C:
   def __init__(self, device=0x42, bus=0):
      self.fr = io.open("/dev/i2c-"+str(bus), "rb", buffering=0)
      self.fw = io.open("/dev/i2c-"+str(bus), "wb", buffering=0)
      # set device address
      fcntl.ioctl(self.fr, I2C_SLAVE, device)
      fcntl.ioctl(self.fw, I2C_SLAVE, device)

   def write(self, data):
      if type(data) is list:
         data = bytearray(data)
      elif type(data) is str:
         data = _b(data)
      self.fw.write(data)
      self.fw.flush()

   def read(self, count):
      return self.fr.read(count)

   def close(self):
      self.fw.close()
      self.fr.close()

def send_data_to_i2c(data):
   global i2c
   sent_str = str(data)
   packer = WirePacker(buffer_size=len(sent_str) + 8)
   for s in sent_str:
      packer.write(ord(s))
   packer.end()
   if packer.available():
      packet = packer.buffer[:packer.available()]
      rospy.loginfo("[{}]: {}".format(rospy.get_name(), packet.decode(errors='replace')))
      try:
         i2c.write(packet)
      except OSError as e:
         rospy.logerr(e)
   time.sleep(0.01)

def eye_status_cb(msg):
   rospy.loginfo("[{}]: eye_status: {}".format(rospy.get_name(), msg.data))
   send_data_to_i2c("eye_status: {}".format(msg.data))

def look_at_cb(msg):
   ropsy.loginfo("[{}]: look_at: {} {}".format(rospy.get_name(), msg.x, msg.y))
   send_data_to_i2c("look_at: {} {}".format(msg.x, msg.y))

def main():
   global i2c
   rospy.init_node('eye_display')

   #  setup i2c settings
   slave = rospy.get_param("i2c_slave", I2C_SLAVE)
   device = int(rospy.get_param("~i2c_device", "0x42"), 16)
   bus = rospy.get_param("~i2c_bus", 0)
   rospy.loginfo("open i2C slave={:#x}, device={:#x}, bus={}".format(slave, device, bus))
   i2c = I2C(device=device, bus=bus)

   # get mode/direction param
   mode_right = rospy.get_param("~mode_right", None)
   if mode_right:
      if isinstance(mode_right, bool):
         send_data_to_i2c("mode_right: {}".format(mode_right))
      else:
         rospy.logwarn("Invalid mode_right ({})".format(mode_right))

   direction = rospy.get_param("~direction", None)
   if direction:
      if isinstance(direction, int):
         send_data_to_i2c("direction: {}".format(direction))
      else:
         rospy.logwarn("Invalid direction ({})".format(direction))

   # get eye_asset param
   eye_asset_names = rospy.get_param("~eye_asset/names")
   send_data_to_i2c("eye_asset_names: {}".format(', '.join(eye_asset_names)))

   for name in eye_asset_names:
      eye_asset_params = rospy.get_param("~eye_asset/{}".format(name))
      for key, value in eye_asset_params.items():
         # image path : cehck if key start with path_
         if key.startswith("path_"):
            eye_type = key[len("path_"):]
            send_data_to_i2c("eye_asset_image_path: {}: {}: {}".format(name, eye_type, value))
         # position*
         if "_position" in key:
            eye_type = key[:key.find("_position")]
            send_data_to_i2c("eye_asset_{}: {}: {}: {}".format(key[len(eye_type)+1:], name, eye_type, value))
         # default_*
         if "_default" in key:
            eye_type = key[:key.find("_default")]
            send_data_to_i2c("eye_default_{}: {}: {}: {}".format(key[len(eye_type)+1:], name, eye_type, value))

   # start subscriber
   rospy.Subscriber('~eye_status', String, eye_status_cb)
   rospy.Subscriber('~look_at', Point, look_at_cb)
   rospy.spin()

if __name__ == "__main__":
    main()



