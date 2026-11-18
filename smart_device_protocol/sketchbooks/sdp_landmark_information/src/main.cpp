#include <vector>
#include <variant>

#if defined(M5STACK_FIRE)
#include <M5Stack.h>
#elif defined(M5STACK_CORE2)
#include <M5Core2.h>
#endif
#include <FS.h>
#include <SPIFFS.h>

#include <ArduinoJson.h>

#include <smart_device_protocol/Packet.h>

#include <sdp/sdp.h>
#include <utils/config_loader.h>
#include <devices/uwb_module_util.h>

// Device name
String device_name = "";

// ESP-NOW
uint8_t mac_address[6] = { 0 };

// SDP Interface
std::string packet_description_information = "Landmark information";
std::string serialization_format_information = "S";
std::vector<SDPData> data_for_information_data_packet;

// UWB
int uwb_id = -1;
std::string packet_description_uwb = "UWB Station";
std::string serialization_format_uwb = "i";
std::vector<SDPData> data_for_uwb_data_packet;

// Other
std::vector<SDPData> data;
int loop_counter = 0;

bool load_config_from_FS(fs::FS& fs, String filename = "/config.json")
{
  StaticJsonDocument<1024> doc;
  if (not load_json_from_FS<1024>(fs, filename, doc))
  {
    return false;
  }
  if (not doc.containsKey("device_name") or not doc.containsKey("uwb_id") or not doc.containsKey("information"))
  {
    return false;
  }

  device_name = doc["device_name"].as<String>();
  uwb_id = doc["uwb_id"].as<int>();
  String information_str = doc["information"].as<String>();
  data_for_information_data_packet.push_back(SDPData(std::string(information_str.c_str())));

  return true;
}

void setup()
{
  M5.begin(true, true, true, false);
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 22, 21);

  // LCD Print
  M5.Lcd.printf("SDP LANDMARK INFORMATION HOST\n");

  // Load config from FS
  SPIFFS.begin();
  SD.begin();
  if (not load_config_from_FS(SD, "/config.json"))
  {
    if (not load_config_from_FS(SPIFFS, "/config.json"))
    {
      Serial.println("Failed to load config file");
      M5.lcd.printf("Failed to load config file\n");
      while (true)
      {
        delay(1000);
      }
    }
  }

  // Initialization of SDP
  if (not init_sdp(mac_address, device_name))
  {
    Serial.println("Failed to initialize SDP");
    M5.lcd.printf("Failed to initialize SDP\n");
    while (true)
    {
      delay(1000);
    }
  }
  Serial.println("SDP Initialized!");

  // UWB module
#if defined(M5STACK_FIRE)
  Serial1.begin(115200, SERIAL_8N1, 16, 17);
#elif defined(M5STACK_CORE2)
  Serial1.begin(115200, SERIAL_8N1, 33, 32);
#endif
  bool result = initUWB(false, uwb_id, Serial1);
  data_for_uwb_data_packet.push_back(SDPData(uwb_id));
  if (result)
  {
    M5.lcd.printf("Success for initialization of UWB\n");
  }
  else
  {
    M5.lcd.printf("Failed to initialize UWB\n");
  }

  // Display MAC address
  M5.Lcd.printf("Name: %s\n", device_name.c_str());
  M5.Lcd.printf("ADDR: %2x:%2x:%2x:%2x:%2x:%2x\n", mac_address[0], mac_address[1], mac_address[2], mac_address[3],
                mac_address[4], mac_address[5]);

  // Display loaded config
  M5.Lcd.printf("UWB ID: %d\n", uwb_id);
}

void loop()
{
  delay(5000);

  // Send SDP data packet
  if (not send_sdp_data_packet(packet_description_information, data_for_information_data_packet))
  {
    Serial.println("Failed to send SDP data packet");
  }
  if (not send_sdp_data_packet(packet_description_uwb, data_for_uwb_data_packet))
  {
    Serial.println("Failed to send SDP data packet");
  }
}
