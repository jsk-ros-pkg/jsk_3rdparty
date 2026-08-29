// See https://qiita.com/nnn112358/items/1cd4517ea2faa57e9c13

#include <WiFi.h>

// Change here
// DO NOT UPLOAD SSID AND PASSWORD
const char* ssid = "*****";
const char* password = "*****";
IPAddress server(192, 168, 10, 100); //ROS core IP adres

void setupWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if (i == 21) {
    Serial.print("Could not connect to "); Serial.println(ssid);
  }
  else {
    Serial.print("Ready! Use ");
    Serial.print(WiFi.localIP());
    Serial.println(" to access client");
  }
}
