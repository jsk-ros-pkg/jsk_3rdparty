#include <variant>

#include <M5EPD.h>

#include "sdp/sdp.h"
#include "utils/config_loader.h"

#include "epd.h"

String device_name = "SDP_DEBUG_BOARD";
unsigned long duration_timeout = 30 * 1000;

// EPD Canvas
M5EPD_Canvas canvas_title(&M5.EPD);
M5EPD_Canvas canvas_info(&M5.EPD);
M5EPD_Canvas canvas_device_interfaces(&M5.EPD);
M5EPD_Canvas canvas_data_frame(&M5.EPD);

uint8_t mac_address[6] = {0};

// SDP Packet Buffer
const int packets_buffer_length = 5;
std::vector<std::tuple<unsigned long, SDPInterfaceDescription, std::vector<SDPData>>> data_packets;

// Other
int loop_counter = 0;

void callback_data_packet(const uint8_t *mac_addr, const SDPInterfaceDescription &interface_description, const std::vector<SDPData> &body)
{
  if (data_packets.size() >= packets_buffer_length)
  {
    data_packets.erase(data_packets.begin());
  }
  data_packets.push_back(std::make_tuple(millis(), interface_description, body));
}

void load_config()
{
  StaticJsonDocument<1024> doc;
  if (not load_json_from_FS<1024>(SD, "/config.json", doc))
  {
    return;
  }
  if (doc.containsKey("device_name"))
    device_name = doc["device_name"].as<String>();
  if (doc.containsKey("timeout_sec"))
    duration_timeout = (unsigned long)(doc["timeout_sec"].as<float>()) * 1000;
}

void setup()
{
  M5.begin(false, true, true, true, false);
  M5.EPD.SetRotation(90);
  M5.EPD.Clear(true);
  M5.RTC.begin();

  // Initialization of EPD
  init_epd(canvas_title, canvas_info, canvas_device_interfaces, canvas_data_frame);
  canvas_title.printf("SDP PACKET PRINTER\n");
  canvas_info.printf("Display initialized!\n");
  update_epd(canvas_title, canvas_info, canvas_device_interfaces, canvas_data_frame);
  Serial.println("Display initialized!");

  // Load config
  load_config();

  // Initialization of SDP
  if (not init_sdp(mac_address, device_name.c_str()))
  {
    Serial.println("SDP Initialization failed!");
    canvas_info.printf("SDP Initialization failed!\n");
    update_epd(canvas_title, canvas_info, canvas_device_interfaces, canvas_data_frame);
    while (true)
    {
      delay(1000);
    }
  }
  register_sdp_data_callback(callback_data_packet);
  Serial.println("SDP Initialized!");

  // Show device info
  canvas_title.printf("Name: %s\n", device_name.c_str());
  canvas_title.printf("ADDR: %2x:%2x:%2x:%2x:%2x:%2x\n",
                      mac_address[0], mac_address[1], mac_address[2],
                      mac_address[3], mac_address[4], mac_address[5]);
  update_epd(canvas_title, canvas_info, canvas_device_interfaces, canvas_data_frame);
}

void loop()
{
  Serial.println("loop");

  // Show Battery voltage
  uint32_t battery_voltage = M5.getBatteryVoltage();
  clear_epd(canvas_info);
  if (loop_counter % 2 == 0)
  {
    canvas_info.printf("+ Battery: %u\n", battery_voltage);
  }
  else
  {
    canvas_info.printf("x Battery: %u\n", battery_voltage);
  }

  // Remove expired data packets
  unsigned long current_time = millis();
  for (auto it = data_packets.begin(); it != data_packets.end();)
  {
    if (current_time - std::get<0>(*it) > duration_timeout)
    {
      it = data_packets.erase(it);
    }
    else
    {
      ++it;
    }
  }

  // Show device interfaces
  clear_epd(canvas_device_interfaces);
  canvas_device_interfaces.printf("Device Interfaces:\n");
  auto device_interfaces = get_sdp_interfaces();
  for (auto it = device_interfaces.begin(); it != device_interfaces.end(); ++it)
  {
    canvas_device_interfaces.printf("  %s : %s\n", std::get<0>(*it).c_str(), std::get<1>(*it).c_str());
    int index = 0;
    for (auto it_if = std::get<2>(*it).begin(); it_if != std::get<2>(*it).end(); ++it_if)
    {
      canvas_device_interfaces.printf("    PD[%d]: %s\n", index, std::get<0>(*it_if).c_str());
      canvas_device_interfaces.printf("    SF[%d]: %s\n", index, std::get<1>(*it_if).c_str());
      index++;
    }
  }

  // Show data packets
  clear_epd(canvas_data_frame);
  canvas_data_frame.printf("Data Packets:\n");
  for (auto it = data_packets.rbegin(); it != data_packets.rend(); ++it)
  {
    const unsigned long timestamp = std::get<0>(*it);
    const SDPInterfaceDescription &interface_description = std::get<1>(*it);
    const std::string packet_description = std::get<0>(interface_description);
    const std::string serialization_format = std::get<1>(interface_description);
    const std::vector<SDPData> &body = std::get<2>(*it);
    canvas_data_frame.printf("=======================\n");
    canvas_data_frame.printf("Time passed: %f sec\n", 1.0 * (millis() - timestamp) / 1000.0);
    canvas_data_frame.printf("Packet Description: %s\n", packet_description.c_str());
    canvas_data_frame.printf("Serialization Format: %s\n", serialization_format.c_str());
    canvas_data_frame.printf("Body:\n");
    for (auto itr = body.rbegin(); itr != body.rend(); itr++)
    {
      if (std::holds_alternative<int32_t>(*itr))
      {
        canvas_data_frame.printf("  %d\n", std::get<int32_t>(*itr));
      }
      else if (std::holds_alternative<float>(*itr))
      {
        canvas_data_frame.printf("  %f\n", std::get<float>(*itr));
      }
      else if (std::holds_alternative<std::string>(*itr))
      {
        canvas_data_frame.printf("  %s\n", std::get<std::string>(*itr).c_str());
      }
      else if (std::holds_alternative<bool>(*itr))
      {
        canvas_data_frame.printf("  %s\n", std::get<bool>(*itr) ? "true" : "false");
      }
      else
      {
        canvas_data_frame.printf("  unknown\n");
      }
    }
  }

  update_epd(canvas_title, canvas_info, canvas_device_interfaces, canvas_data_frame);
  ++loop_counter;
}