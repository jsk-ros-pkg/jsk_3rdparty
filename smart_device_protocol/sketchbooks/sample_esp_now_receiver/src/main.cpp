#define LGFX_M5STACK
#define LGFX_USE_V1
#include <M5Stack.h>
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>

#include <esp_system.h>
#include <esp_now.h>
#include <WiFi.h>

static LGFX lcd;
static LGFX_Sprite sprite_device_info(&lcd);
static LGFX_Sprite sprite_packet_info(&lcd);

uint8_t mac_address[6] = {0};

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  sprite_packet_info.fillScreen(0xFFFFFF);
  sprite_packet_info.setCursor(0, 0);
  sprite_packet_info.print("Last Packet Recv from: ");
  sprite_packet_info.printf("%02X:%02X:%02X:%02X:%02X:%02X\n", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  sprite_packet_info.printf("Last Packet Recv Data(%d) (HEX): ", data_len);
  for ( int i = 0; i < data_len; i++ ) {
      sprite_packet_info.printf("%#d ", data[i]);
  }
  sprite_packet_info.println("");
  sprite_packet_info.printf("Last Packet Recv Data(%d) (string): ", data_len);
  for ( int i = 0; i < data_len; i++ ) {
    sprite_packet_info.printf("%c", (char)data[i]);
  }
  sprite_packet_info.println("");
}

void setup() {
  Serial.begin(115200);

  esp_read_mac(mac_address, ESP_MAC_WIFI_STA);

  lcd.init();
  lcd.setRotation(1);
  lcd.setBrightness(128);
  lcd.setColorDepth(24);
  lcd.fillScreen(0xFFFFFF);

  sprite_device_info.createSprite(320, 50);
  sprite_packet_info.createSprite(320, 150);

  sprite_device_info.fillScreen(0xFFFFFF);
  sprite_device_info.setTextColor(0x000000);
  sprite_device_info.setTextSize(1.5, 1.5);
  sprite_packet_info.fillScreen(0xFFFFFF);
  sprite_packet_info.setTextColor(0x000000);
  //sprite_packet_info.setTextSize(1.5, 1.5)

  sprite_device_info.println("ESP_NOW broadcast example");
  sprite_device_info.printf(
          "MAC: %02X:%02X:%02X:%02X:%02X:%02X",
          mac_address[0],
          mac_address[1],
          mac_address[2],
          mac_address[3],
          mac_address[4],
          mac_address[5]
          );
  sprite_device_info.pushSprite(0, 0);
  sprite_packet_info.pushSprite(0, 50);

  // ESP-NOW初期化
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (not esp_now_init() == ESP_OK) {
    ESP.restart();
  }

  // Register Callback
  esp_now_register_recv_cb(OnDataRecv);

  lcd.startWrite();
}
void loop() {
  sprite_packet_info.pushSprite(0, 50);
  Serial.println("Update LCD!");
  delay(100);
}
