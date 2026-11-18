// Please see
// https://www.mgo-tec.com/blog-entry-chip-info-esp-wroom-32-esp32.html

void esp_info() {
  Serial.println("-----------------------------");

  uint8_t mac5[6];
  esp_read_mac(mac5, ESP_MAC_BT);
  Serial.printf("[Bluetooth] Mac Address = %02X:%02X:%02X:%02X:%02X:%02X\r\n", mac5[0], mac5[1], mac5[2], mac5[3], mac5[4], mac5[5]);

  uint8_t mac6[6];
  esp_read_mac(mac6, ESP_MAC_ETH);
  Serial.printf("[Ethernet] Mac Address = %02X:%02X:%02X:%02X:%02X:%02X\r\n", mac6[0], mac6[1], mac6[2], mac6[3], mac6[4], mac6[5]);

  Serial.println("-----------------------------");
}
