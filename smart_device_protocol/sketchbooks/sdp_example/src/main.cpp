#include <vector>

#include "M5Stack.h"

#define LGFX_M5STACK
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>

#include "sdp/sdp_util.h"

#include "lcd.h"

// Device name
String device_name = "sdp_sample";

// LovyanGFX
static LGFX lcd;
static LGFX_Sprite sprite_device_info(&lcd);
static LGFX_Sprite sprite_send_info(&lcd);
static LGFX_Sprite sprite_recieve_info(&lcd);

// SDP
uint8_t mac_address[6] = {0};

// SDPInterface Example
SDPInterfaceDescription sample_interface_description = std::make_tuple<std::string, std::string>("test interface", "sif?");

// SDPData Example
std::vector<SDPData> body_data;

void callback_sdp_sample(std::vector<SDPData> &body)
{
  Serial.println("Callback: sdp_sample");
  Serial.printf("Received SDP Data for interface %s\n", std::get<0>(sample_interface_description).c_str());
  clear_sprite(sprite_recieve_info);
  sprite_recieve_info.println("Received SDP Data for interface");
  print_sdp_body(sprite_recieve_info, body);
  body_data[0] = body[0];
  body_data[1] = body[1];
  body_data[2] = body[2];
  body_data[3] = body[3];
}

void setup()
{
  M5.begin(false, true, true, false);
  Serial.begin(115200);

  // Initialize LCD
  init_lcd(lcd, sprite_device_info, sprite_send_info, sprite_recieve_info);
  sprite_device_info.println("Smart Device Protocol Broadcast Example");
  sprite_device_info.println("Initializing...");
  update_lcd(sprite_device_info, sprite_send_info, sprite_recieve_info);

  // Initialize SDP
  if (not init_sdp(mac_address, String("test")))
  {
    Serial.println("Failed to initialize SDP");
    sprite_device_info.println("Failed to initialize SDP");
    update_lcd(sprite_device_info, sprite_send_info, sprite_recieve_info);
    while (true)
    {
      delay(1000);
    }
  }
  else
  {
    Serial.println("Initialized SDP");
    sprite_device_info.println("Initialized SDP");
    update_lcd(sprite_device_info, sprite_send_info, sprite_recieve_info);
  }

  // Update Info Screen
  sprite_device_info.printf("Device Name: %s", device_name.c_str());
  sprite_device_info.printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                            mac_address[0], mac_address[1], mac_address[2],
                            mac_address[3], mac_address[4], mac_address[5]);
  update_lcd(sprite_device_info, sprite_send_info, sprite_recieve_info);

  // Packet
  body_data.clear();
  body_data.push_back(SDPData(std::string("Hello, World!")));
  body_data.push_back(SDPData((int32_t)123));
  body_data.push_back(SDPData((float)123.456));
  body_data.push_back(SDPData(true));
}

void loop()
{
  delay(1000);
  if (not send_sdp_data_packet(sample_interface_description, body_data))
  {
    Serial.println("Failed to send SDP Data Body");
    clear_sprite(sprite_send_info);
    sprite_send_info.println("Failed to send SDP Data Body.");
    update_lcd(sprite_device_info, sprite_send_info, sprite_recieve_info);
  }
  else
  {
    Serial.println("Sent SDP Data Body");
    clear_sprite(sprite_send_info);
    sprite_send_info.println("Sent SDP Data Body");
    print_sdp_body(sprite_send_info, body_data);
    update_lcd(sprite_device_info, sprite_send_info, sprite_recieve_info);
  }
}
