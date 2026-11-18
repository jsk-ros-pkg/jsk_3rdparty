#include <smart_device_protocol/Packet.h>

#ifdef USE_DISPLAY
#define LGFX_AUTODETECT
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>

// Lovyan GFX
static inline LGFX lcd;
static inline LGFX_Sprite sprite_device_info(&lcd);
static inline LGFX_Sprite sprite_device_status(&lcd);
static inline LGFX_Sprite sprite_event_info(&lcd);

void init_lcd()
{
  lcd.init();
  lcd.setRotation(1);
  lcd.setBrightness(128);
  lcd.setColorDepth(24);
  lcd.fillScreen(0xFFFFFF);

  sprite_device_info.createSprite(lcd.width(), lcd.height() / 3);
  sprite_device_status.createSprite(lcd.width(), lcd.height() / 3);
  sprite_event_info.createSprite(lcd.width(), lcd.height() / 3);

  sprite_device_info.fillScreen(0xFFFFFF);
  sprite_device_info.setTextColor(0x000000);
  sprite_device_status.fillScreen(0xFFFFFF);
  sprite_device_status.setTextColor(0x000000);
  sprite_event_info.fillScreen(0xFFFFFF);
  sprite_event_info.setTextColor(0x000000);

#if defined(M5STACKATOMS3)
  sprite_device_info.setTextSize(1.0, 1.0);
  sprite_device_status.setTextSize(1.0, 1.0);
  sprite_event_info.setTextSize(1.0, 1.0);
#else
  sprite_device_info.setTextSize(1.5, 1.5);
#endif
}

void _clear_lcd(LGFX_Sprite& sprite)
{
  sprite.fillScreen(0xFFFFFF);
  sprite.setCursor(0, 0);
}

void clear_device_info()
{
  _clear_lcd(sprite_device_info);
}

void clear_device_status()
{
  _clear_lcd(sprite_device_status);
}

void clear_event_info()
{
  _clear_lcd(sprite_event_info);
}

template <typename T>
void print_event_info(T info)
{
  sprite_event_info.println(info);
}

void _update_lcd(LGFX& lcd, LGFX_Sprite& sprite_device_info, LGFX_Sprite& sprite_device_status,
                 LGFX_Sprite& sprite_event_info)
{
  sprite_device_info.pushSprite(0, 0);
  sprite_device_status.pushSprite(0, lcd.height() / 3);
  sprite_event_info.pushSprite(0, lcd.height() * 2 / 3);
}

void update_lcd()
{
  _update_lcd(lcd, sprite_device_info, sprite_device_status, sprite_device_info);
}

void print_ros_message_info(const smart_device_protocol::Packet& msg)
{
  sprite_event_info.printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", msg.mac_address[0], msg.mac_address[1],
                           msg.mac_address[2], msg.mac_address[3], msg.mac_address[4], msg.mac_address[5]);
  sprite_event_info.print("data: ");
  for (int i = 0; i < msg.data_length; i++)
  {
    sprite_event_info.printf("%d ", msg.data[i]);
  }
  sprite_event_info.println("");
}

void print_device_info(const uint8_t* device_mac_address, bool uwb_enabled, int tag_id)
{
  sprite_device_info.println("ESP-NOW ROS Driver");
  sprite_device_info.printf("MAC ADDR: %02X:%02X:%02X:%02X:%02X:%02X\n", device_mac_address[0], device_mac_address[1],
                            device_mac_address[2], device_mac_address[3], device_mac_address[4], device_mac_address[5]);
  sprite_device_info.printf("UWB Enabled: %s\n", uwb_enabled ? "true" : "false");
  sprite_device_info.printf("Tag ID: %d\n", tag_id);
}

#else

void init_lcd()
{
}

void clear_device_info()
{
}

void clear_device_status()
{
}

void clear_event_info()
{
}

template <typename T>
void print_event_info(T info)
{
}

void update_lcd()
{
}

void print_ros_message_info(const smart_device_protocol::Packet& msg)
{
}

void print_device_info(const uint8_t* device_mac_address, bool uwb_enabled, int tag_id)
{
}

#endif