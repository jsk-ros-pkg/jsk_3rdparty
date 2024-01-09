#include <esp_now.h>
#include <esp_system.h>

#include <Arduino.h>
#include <WiFi.h>

#define LGFX_AUTODETECT
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>

#include "devices/uwb_module_util.h"

static LGFX lcd;
static LGFX_Sprite sprite_device_info(&lcd);
static LGFX_Sprite sprite_event_info(&lcd);

void setup()
{
  // Read device mac address
  uint8_t device_mac_address[6] = {0};
  esp_read_mac(device_mac_address, ESP_MAC_WIFI_STA);

  // LCD Initialization
  lcd.init();
  lcd.setRotation(1);
  lcd.setBrightness(128);
  lcd.setColorDepth(24);
  lcd.fillScreen(0xFFFFFF);

  sprite_device_info.createSprite(lcd.width(), lcd.height() / 3);
  sprite_event_info.createSprite(lcd.width(), lcd.height() * 2 / 3);

  sprite_device_info.fillScreen(0xFFFFFF);
  sprite_device_info.setTextColor(0x000000);
  sprite_device_info.setTextSize(1.0, 1.0);
  sprite_event_info.setTextSize(1.0, 1.0);
  sprite_event_info.fillScreen(0xFFFFFF);
  sprite_event_info.setTextColor(0x000000);

  sprite_device_info.println("SDP UWB Sample");
  sprite_device_info.printf("MAC ADDR: %02X:%02X:%02X:%02X:%02X:%02X\n", device_mac_address[0], device_mac_address[1],
                            device_mac_address[2], device_mac_address[3], device_mac_address[4], device_mac_address[5]);
  sprite_device_info.pushSprite(0, 0);

  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 1, 2);
  delay(100);

  // initUWB(true, 0);
  initUWB(false, 1, Serial2);
}

void loop()
{
  delay(100);
  auto ret = readUWB(Serial2);
  if (ret)
  {
    Serial.println(*ret);

    sprite_event_info.fillScreen(0xFFFFFF);
    sprite_event_info.setCursor(0, 0);
    sprite_event_info.println(*ret);
    sprite_event_info.pushSprite(0, lcd.height() / 3);
  }
  else
  {
    Serial.println("No response");

    sprite_event_info.fillScreen(0xFFFFFF);
    sprite_event_info.setCursor(0, 0);
    sprite_event_info.println("No response");
    sprite_event_info.pushSprite(0, lcd.height() / 3);
  }
}
