// Arduino
#if defined(M5STACKFIRE) or defined(M5STACKCORE2)
#include <Arduino.h>
#define LGFX_USE_V1
#else
#include <Arduino.h>
#endif

// LCD
#include "lcd.h"

// ROS
#include "ros/node_handle.h"
#include <smart_device_protocol/Packet.h>
#include <smart_device_protocol/UWBDistance.h>
#if defined(M5STACKATOMS3)
#include "ArduinoAtomS3Hardware.h"
#else
#include "ArduinoHardware.h"
#endif

#include <WiFi.h>
#include "sdp/esp_now.h"
#include "devices/uwb_module_util.h"

void messageCb(const smart_device_protocol::Packet &);

// ESP-NOW
uint8_t device_mac_address[6] = { 0 };
uint8_t mac_address_for_msg[6];
uint8_t buffer_for_msg[256];

// UWB
int8_t uwb_serial_rx_pin = 1;
int8_t uwb_serial_tx_pin = 2;
bool uwb_initialized = false;

// ROSSerial
smart_device_protocol::Packet msg_recv_packet;
smart_device_protocol::UWBDistance msg_uwb;
ros::NodeHandle_<ArduinoHardware> nh;
ros::Publisher publisher("/smart_device_protocol/recv", &msg_recv_packet);
ros::Publisher publisher_uwb("/smart_device_protocol/uwb", &msg_uwb);
ros::Subscriber<smart_device_protocol::Packet> subscriber("/smart_device_protocol/send", &messageCb);

void messageCb(const smart_device_protocol::Packet &msg)
{
  if (msg.mac_address_length != 6)
  {
    nh.logerror("MAC Address length have to be 6.");
    return;
  }

  // Register a peer
  esp_now_peer_info_t peer_temp;
  memset(&peer_temp, 0, sizeof(peer_temp));
  for (int i = 0; i < 6; i++)
  {
    peer_temp.peer_addr[i] = msg.mac_address[i];
  }
  esp_err_t add_status = esp_now_add_peer(&peer_temp);
  esp_err_t result = esp_now_send(peer_temp.peer_addr, (uint8_t*)msg.data, msg.data_length);
  esp_err_t del_status = esp_now_del_peer(peer_temp.peer_addr);

  // Display
  clear_event_info();
  print_event_info("Send a packet");
  print_ros_message_info(msg);
  update_lcd();

  // ROS Logging
  nh.logdebug("Subscribe a message and send a packet.");
}

void OnDataRecv(const uint8_t* mac_addr, const uint8_t* data, int data_len)
{
  for (int i = 0; i < 6; i++)
  {
    mac_address_for_msg[i] = mac_addr[i];
  }
  for (int i = 0; i < data_len; i++)
  {
    buffer_for_msg[i] = data[i];
  }
  msg_recv_packet.mac_address = mac_address_for_msg;
  msg_recv_packet.mac_address_length = 6;
  msg_recv_packet.data = buffer_for_msg;
  msg_recv_packet.data_length = data_len;
  publisher.publish(&msg_recv_packet);
  nh.spinOnce();

  // Display
  clear_event_info();
  print_event_info("Receive a packet");
  print_ros_message_info(msg_recv_packet);
  update_lcd();

  // Log
  nh.logdebug("Received a packet and publish a message.");
}

void setup()
{
  // UWB initialization
  Serial2.begin(115200, SERIAL_8N1, uwb_serial_rx_pin, uwb_serial_tx_pin);

  // Rosserial Initialization
  nh.initNode();
  while (not nh.connected())
  {
    delay(1000);
    nh.spinOnce();
  }

  int tag_id = 0;
  nh.getParam("~tag_id", &tag_id, 1);

  // UWB initialization
  uwb_initialized = initUWB(true, tag_id, Serial2);

  // Subscribe and Publish
  nh.advertise(publisher);
  if (uwb_initialized)
  {
    nh.advertise(publisher_uwb);
  }
  nh.subscribe(subscriber);
  while (not nh.connected())
  {
    delay(1000);
    nh.spinOnce();
  }

  // ESP-NOW initialization
  if (not init_esp_now(device_mac_address, OnDataRecv))
  {
    while (true)
    {
      delay(1000);
      nh.logerror("ESPNow Init Failed");
    }
  }

  // Log
  nh.loginfo("ESPNow Init Success");
  if (uwb_initialized)
  {
    char buf_str[128];
    sprintf(buf_str, "UWB Init Success. Tag ID: %d", tag_id);
    nh.loginfo(buf_str);
  }
  else
  {
    nh.loginfo("UWB Init Failed. Disabled.");
  }

  // LCD Initialization
  init_lcd();

  // Display
  clear_device_info();
  print_device_info(device_mac_address, uwb_initialized, tag_id);
  update_lcd();
}

void loop()
{
  if (uwb_initialized)
  {
    auto ret = getDistanceUWB(Serial2);
    if (ret)
    {
      char buf_for_log[256];
      int id = std::get<0>(*ret);
      float distance = std::get<1>(*ret);
      msg_uwb.header.stamp = nh.now();
      msg_uwb.id = id;
      msg_uwb.distance = distance;
      publisher_uwb.publish(&msg_uwb);
      sprintf(buf_for_log, "id: %d, distance: %f", id, distance);
      nh.logdebug(buf_for_log);
    }
  }
  nh.spinOnce();
  delay(100);
}
