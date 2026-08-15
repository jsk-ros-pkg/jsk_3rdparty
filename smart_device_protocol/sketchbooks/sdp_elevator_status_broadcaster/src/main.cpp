#include <M5Core2.h>

#include <WiFi.h>

#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>
#include <Dps310.h>
#include <ArduinoJson.h>

#include "smart_device_protocol/Packet.h"
#include "sdp/sdp.h"
#include "utils/config_loader.h"

static LGFX lcd;
static LGFX_Sprite sprite_header(&lcd);
static LGFX_Sprite sprite_status(&lcd);

Dps310 Dps310PressureSensor = Dps310();

/* Sensor */
float sensor_accX = 0.0F;
float sensor_accY = 0.0F;
float sensor_accZ = 0.0F;
float sensor_gyroX = 0.0F;
float sensor_gyroY = 0.0F;
float sensor_gyroZ = 0.0F;
float sensor_pitch = 0.0F;
float sensor_roll = 0.0F;
float sensor_yaw = 0.0F;
float sensor_temp_mpu = 0.0F;
float sensor_temp_dps = 0;
float sensor_pressure = 0;

/* device */
uint8_t device_mac_address[6];
String device_name;

/* Elevator config */
typedef struct
{
  uint8_t floor_num;
  float floor_height;
} ElevatorConfig;
std::vector<ElevatorConfig> elevator_config;

bool load_config_from_FS(fs::FS &fs, const String &filename)
{
  StaticJsonDocument<1024> doc;
  if (!load_json_from_FS<1024>(fs, filename, doc))
  {
    return false;
  }

  if (not doc.containsKey("device_name") or not doc.containsKey("elevator_config"))
  {
    return false;
  }

  device_name = doc["device_name"].as<String>();
  JsonArray elevator_config_json = doc["elevator_config"].as<JsonArray>();
  for (auto itr = elevator_config_json.begin(); itr != elevator_config_json.end(); ++itr)
  {
    JsonObject e = *itr;
    if (e.containsKey("floor_num") and e.containsKey("floor_height"))
    {
      ElevatorConfig ec;
      ec.floor_num = e["floor_num"].as<uint8_t>();
      ec.floor_height = e["floor_height"].as<float>();
      elevator_config.push_back(ec);
    }
  }
  return true;
}

void init_lcd()
{
  // LCD
  lcd.init();
  lcd.setRotation(1);
  lcd.setBrightness(128);
  lcd.setColorDepth(24);
  lcd.fillScreen(0xFFFFFF);

  sprite_header.createSprite(lcd.width(), lcd.height() / 4);
  sprite_status.createSprite(lcd.width(), lcd.height() * 3 / 4);

  sprite_header.fillScreen(0xFFFFFF);
  sprite_header.setTextColor(0x000000);
  sprite_header.setTextSize(1.5, 1.5);
  sprite_status.fillScreen(0xFFFFFF);
  sprite_status.setTextColor(0x000000);
}

void measure_sensors()
{
  M5.IMU.getGyroData(&sensor_gyroX, &sensor_gyroY, &sensor_gyroZ);
  M5.IMU.getAccelData(&sensor_accX, &sensor_accY, &sensor_accZ);
  M5.IMU.getAhrsData(&sensor_pitch, &sensor_roll, &sensor_yaw);
  M5.IMU.getTempData(&sensor_temp_mpu);
  Dps310PressureSensor.measurePressureOnce(sensor_pressure);
  Dps310PressureSensor.measureTempOnce(sensor_temp_dps);
}

void setup()
{
  // Device Initialization
  M5.begin(true, false, true, true);
  Serial.begin(115200);
  M5.IMU.Init();
  Dps310PressureSensor.begin(Wire, 0x77);

  // initialize LCD
  init_lcd();

  // Load config
  SD.begin();
  SPIFFS.begin();
  if (not load_config_from_FS(SD, "/config.json"))
  {
    Serial.println("Failed to load config from SD");
    if (not load_config_from_FS(SPIFFS, "/config.json"))
    {
      Serial.println("Failed to load config from SPIFFS");
      while (true)
      {
        delay(1000);
      }
    }
  }

  // Initialize ESP-NOW
  init_sdp(device_mac_address, String("elevator_status"));

  // Print
  sprite_header.println("ENR DPS310 Interface");
  sprite_header.printf("MAC ADDR: %02X:%02X:%02X:%02X:%02X:%02X\n",
                       device_mac_address[0], device_mac_address[1], device_mac_address[2],
                       device_mac_address[3], device_mac_address[4], device_mac_address[5]);
  sprite_header.pushSprite(0, 0);
}

void loop()
{
  measure_sensors();
}
