#include <esp_now.h>
#include <esp_system.h>

#include <WiFi.h>

#include <M5Core2.h>
#include <Dps310.h>

#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>

#include <smart_device_protocol/Packet.h>

#include "sdp/packet_creator.h"
#include "sdp/packet_parser.h"

static LGFX lcd;
static LGFX_Sprite sprite_header(&lcd);
static LGFX_Sprite sprite_sensor(&lcd);

Dps310 Dps310PressureSensor = Dps310();

// SDP
esp_now_peer_info_t peer;
uint8_t packet[240];
String packet_description_config = "Elevator status config";
String serialization_format_config = "S";

// Sensor values
float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;
float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;
float pitch = 0.0F;
float roll = 0.0F;
float yaw = 0.0F;
float temp_mpu = 0.0F;
float temp_dps = 0;
float pressure = 0;

char module_name[64];

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  uint16_t packet_type = get_packet_type(data);
  if (packet_type == smart_device_protocol::Packet::PACKET_TYPE_NAMED_STRING)
  {
    char name[64];
    char temp_module_name[64];
    parse_packet_as_named_string_packet(data, packet_type, name, temp_module_name);
    if (strncmp(name, "module_name", 10))
    {
      strncpy(module_name, temp_module_name, 64);
      Serial.printf("module_name is updated to %s\n", module_name);
    }
  }
}

void setup()
{
  // Read device mac address
  uint8_t device_mac_address[6] = {0};
  esp_read_mac(device_mac_address, ESP_MAC_WIFI_STA);

  // Device Initialization
  M5.begin(true, false, true, true);
  Serial.begin(115200);
  M5.IMU.Init();
  Dps310PressureSensor.begin(Wire, 0x77);

  // Set default name
  strncpy(module_name, "default", 64);

  // LCD
  lcd.init();
  lcd.setRotation(1);
  lcd.setBrightness(128);
  lcd.setColorDepth(24);
  lcd.fillScreen(0xFFFFFF);

  sprite_header.createSprite(320, 80);
  sprite_sensor.createSprite(320, 160);

  sprite_header.fillScreen(0xFFFFFF);
  sprite_header.setTextColor(0x000000);
  sprite_header.setTextSize(1.5, 1.5);
  sprite_sensor.fillScreen(0xFFFFFF);
  sprite_sensor.setTextColor(0x000000);

  sprite_header.println("ENR DPS310 Interface");
  sprite_header.printf("MAC ADDR: %02X:%02X:%02X:%02X:%02X:%02X\n", device_mac_address[0], device_mac_address[1],
                       device_mac_address[2], device_mac_address[3], device_mac_address[4], device_mac_address[5]);
  sprite_header.pushSprite(0, 0);

  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK)
  {
    Serial.println("ESPNow Init Success");
  }
  else
  {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
  esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  M5.IMU.getAccelData(&accX, &accY, &accZ);
  M5.IMU.getAhrsData(&pitch, &roll, &yaw);
  M5.IMU.getTempData(&temp_mpu);

  Dps310PressureSensor.measurePressureOnce(pressure);
  Dps310PressureSensor.measureTempOnce(temp_dps);

  sprite_sensor.fillScreen(0xFFFFFF);
  sprite_sensor.setCursor(0, 0);
  Serial.println("==== measured ====");
  Serial.printf("acc: %f %f %f\n", accX, accY, accZ);
  sprite_sensor.printf("acc: %f %f %f\n", accX, accY, accZ);
  Serial.printf("gyro: %f %f %f\n", gyroX, gyroY, gyroZ);
  sprite_sensor.printf("gyro: %f %f %f\n", gyroX, gyroY, gyroZ);
  Serial.printf("rpy: %f %f %f\n", roll, pitch, yaw);
  sprite_sensor.printf("rpy: %f %f %f\n", roll, pitch, yaw);
  Serial.printf("temp(mpu): %f\n", temp_mpu);
  sprite_sensor.printf("temp(mpu): %f\n", temp_mpu);
  Serial.printf("pressure: %f\n", pressure);
  sprite_sensor.printf("pressure: %f\n", pressure);
  Serial.printf("temp(dps): %f\n", temp_dps);
  sprite_sensor.printf("temp(dps): %f\n", temp_dps);
  sprite_sensor.pushSprite(0, 80);

  create_sensor_enviii_packet(packet, module_name, pressure);
  esp_err_t result = esp_now_send(peer.peer_addr, (uint8_t *)packet, sizeof(packet) / sizeof(packet[0]));
}
