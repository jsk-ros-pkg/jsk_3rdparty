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
static LGFX_Sprite sprite_send_result(&lcd);

uint8_t mac_address[6] = {0};
const char message[] = "Hello, World!";
esp_now_peer_info_t peer_broadcast;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  sprite_packet_info.fillScreen(0xFFFFFF);
  sprite_packet_info.setCursor(0, 0);
  sprite_packet_info.print("Last Packet Sent to: ");
  sprite_packet_info.printf("%02X:%02X:%02X:%02X:%02X:%02X\n", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  sprite_packet_info.print("Last Packet Send Status: ");
  sprite_packet_info.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);

  esp_read_mac(mac_address, ESP_MAC_WIFI_STA);

  lcd.init();
  lcd.setRotation(1);
  lcd.setBrightness(128);
  lcd.setColorDepth(24);
  lcd.fillScreen(0xFFFFFF);

  sprite_device_info.createSprite(300, 50);
  sprite_packet_info.createSprite(300, 50);
  sprite_send_result.createSprite(300, 50);

  sprite_device_info.fillScreen(0xFFFFFF);
  sprite_device_info.setTextColor(0x000000);
  sprite_packet_info.fillScreen(0xFFFFFF);
  sprite_packet_info.setTextColor(0x000000);
  sprite_send_result.fillScreen(0xFFFFFF);
  sprite_send_result.setTextColor(0x000000);

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

  sprite_packet_info.pushSprite(0, 60);
  sprite_send_result.pushSprite(0, 120);

  // ESP-NOW初期化
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (not esp_now_init() == ESP_OK) {
    ESP.restart();
  }

  memset(&peer_broadcast, 0, sizeof(peer_broadcast));
  for (int i = 0; i < 6; ++i) {
    peer_broadcast.peer_addr[i] = (uint8_t)0xff;
  }
  esp_err_t addStatus = esp_now_add_peer(&peer_broadcast);

  esp_now_register_send_cb(OnDataSent);
}
void loop() {
  Serial.println("Broadcasted!");
  esp_err_t result = esp_now_send(peer_broadcast.peer_addr, (uint8_t*)message, sizeof(message));
  sprite_send_result.fillScreen(0xFFFFFF);
  sprite_send_result.setCursor(0, 0);
  sprite_send_result.print("Send Status: ");
  if (result == ESP_OK) {
    sprite_send_result.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    sprite_send_result.println("ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    sprite_send_result.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    sprite_send_result.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    sprite_send_result.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    sprite_send_result.println("Peer not found.");
  } else {
    sprite_send_result.println("Not sure what happened");
  }
  sprite_packet_info.pushSprite(0, 60);
  sprite_send_result.pushSprite(0, 120);
  delay(5000);
}
