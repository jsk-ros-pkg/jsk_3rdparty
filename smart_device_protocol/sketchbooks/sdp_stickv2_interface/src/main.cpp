#include <M5Core2.h>

#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>
#include <ArduinoJson.h>

#include "sdp/sdp.h"
#include "utils/config_loader.h"
#include "devices/stickv2_util.h"
#include "devices/uwb_module_util.h"

#include "lcd.h"

// LovyanGFX
static LGFX lcd;
static LGFX_Sprite sprite_title(&lcd);
static LGFX_Sprite sprite_status(&lcd);
static LGFX_Sprite sprite_info(&lcd);

/* SDP Interface */
std::string packet_description = "Number of object";
std::string serialization_format = "i";

/* device information */
uint8_t mac_address[6];
String device_name;
bool auto_start = true;

// Object Detection Threashold
float threashold = 0.5;
String target_class = "person";

/* UWB */
int uwb_id = -1;

bool load_config_from_FS(fs::FS &fs, const String &filename)
{
  StaticJsonDocument<1024> doc;
  if (not load_json_from_FS<1024>(fs, filename, doc))
  {
    return false;
  }
  if (not doc.containsKey("device_name") or
      not doc.containsKey("uwb_id") or
      not doc.containsKey("threshold") or
      not doc.containsKey("target_class"))
  {
    return false;
  }
  device_name = doc["device_name"].as<String>();
  uwb_id = doc["uwb_id"].as<int>();
  threashold = doc["threshold"].as<float>();
  target_class = doc["target_class"].as<String>();

  if (doc.containsKey("auto_start"))
  {
    auto_start = doc["auto_start"].as<bool>();
  }
  return true;
}

void setup()
{
  // Initialize
  M5.begin(true, true, true, false);
  Serial1.begin(115200, SERIAL_8N1, 33, 32);
  Serial.begin(115200, SERIAL_8N1);

  // Display Initialization
  init_lcd(lcd, sprite_title, sprite_status, sprite_info);

  // Display Title
  sprite_title.println("SDP StickV2 Interface");
  update_lcd(lcd, sprite_title, sprite_status, sprite_info);

  // Load config
  SD.begin();
  SPIFFS.begin();
  if (not load_config_from_FS(SD, String("/config.json")))
  {
    if (not load_config_from_FS(SPIFFS, String("/config.json")))
    {
      Serial.println("Failed to load config.");
      sprite_status.println("Failed to load config.");
      update_lcd(lcd, sprite_title, sprite_status, sprite_info);
      while (true)
      {
        delay(1000);
      }
    }
  }

  // UWB Initialization
  if (not initUWB(false, uwb_id, Serial2))
  {
    Serial.println("UWB Initialization Failed.");
    sprite_status.println("UWB Initialization Failed.");
    update_lcd(lcd, sprite_title, sprite_status, sprite_info);
  }

  // SDP Initialization
  if (not init_sdp(mac_address, device_name.c_str()))
  {
    Serial.println("SDP Initialization Failed.");
    sprite_status.println("SDP Initialization Failed.");
    update_lcd(lcd, sprite_title, sprite_status, sprite_info);
    while (true)
    {
      delay(1000);
    }
  }

  // Display Device Info
  sprite_title.printf("Device Name: %s\n", device_name.c_str());
  sprite_title.printf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      mac_address[0], mac_address[1], mac_address[2],
                      mac_address[3], mac_address[4], mac_address[5]);
  sprite_title.printf("UWB ID: %d\n", uwb_id);
  sprite_title.printf("Target Class: %s\n", target_class.c_str());
  sprite_title.printf("Threashold: %f\n", threashold);
  sprite_status.println("Initialization Completed.");
  update_lcd(lcd, sprite_title, sprite_status, sprite_info);
}

void loop()
{
  auto last_read_stamp = millis();
  StaticJsonDocument<2048> doc;
  while (true)
  {
    delay(100);
    if (Serial.available())
    {
      String cmd = Serial.readStringUntil('\n');
      if (cmd == "start")
      {
        auto_start = true;
      }
      else
      {
        Serial1.print(cmd);
        String response = Serial1.readStringUntil('\n');
        Serial.printf("Response: %s\n", response.c_str());
      }
    }
    if (not Serial1.available() and (millis() - last_read_stamp > 10000) and auto_start)
    {
      set_object_recognition_model(Serial1, String("./uploads/models/nanodet_80class"));
      Serial.println("Set objection dection mode.");
      clear_sprite(sprite_info);
      sprite_info.println("Set objection dection mode.");
      last_read_stamp = millis();
    }
    else if (Serial1.available())
    {
      doc.clear();
      bool success = read_data_from_serial(Serial1, doc);
      String doc_str;
      serializeJson(doc, doc_str);
      clear_sprite(sprite_info);
      Serial.printf("Read doc data: %s\n", doc_str.c_str());
      sprite_info.printf("Read doc data: %s\n", doc_str.c_str());
      if (success and doc.containsKey("num") and doc.containsKey("obj"))
      {
        int num_of_target = 0;
        long num_of_objects = doc["num"];
        for (int i = 0; i < num_of_objects; i++)
        {
          Serial.printf(" %d th object: %s, %f\n",
                        i,
                        doc["obj"][i]["type"].as<String>().c_str(),
                        doc["obj"][i]["prob"].as<float>());
          if (doc["obj"][i]["type"] == target_class and doc["obj"][i]["prob"].as<float>() > threashold)
          {
            ++num_of_target;
          }
        }
        std::vector<SDPData> data;
        data.push_back(SDPData(num_of_target));
        send_sdp_data_packet(packet_description, data);
        clear_sprite(sprite_status);
        sprite_status.printf("Send SDP packet: %d\n", num_of_target);
        Serial.printf("Send SDP packet: %d\n", num_of_target);
      }
      else
      {
        clear_sprite(sprite_status);
        sprite_status.println("Failed to read data.");
      }
      last_read_stamp = millis();
    }
    update_lcd(lcd, sprite_title, sprite_status, sprite_info);
  }
}
