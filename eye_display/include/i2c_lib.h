#pragma once

#include <Wire.h>
#include <WireSlave.h>
#include "util.h"

constexpr int SDA_PIN = 8;
constexpr int SCL_PIN = 9;
#ifdef  I2C_SLAVE_ADDR
constexpr int I2C_SLAVE_ADDR = I2C_SLAVE_ADDR;
#else
constexpr int I2C_SLAVE_ADDR = 0x42;
#endif

extern EyeManager eye;

void receiveEvent(int howMany) {
  std::string message;
  while (0 < WireSlave.available()) {
    char c = WireSlave.read();  // receive byte as a character
    message += c;
  }
  logdebug("I2C received : %s", message.c_str());

  std::string key, value;
  splitKeyValue(message, key, value);
  if ( key  == "eye_status" ) {
    std::map<std::string, EyeAsset>& eye_asset_map = eye.eye_asset_map;
    eye.set_emotion(value);
  } else if ( key  == "look_at") {
    double x, y;
    parseXY(value, x, y);
    eye.set_gaze_direction(x, y);
  } else {
    eye.setup_asset({message});
  }
  return;
}

void I2CTask(void *parameter) {
  bool success = WireSlave.begin(SDA_PIN, SCL_PIN, I2C_SLAVE_ADDR);

  Serial.println("I2C slave start");
  if (!success) {
    Serial.println("I2C slave init failed");
    while (1) delay(100);
  }
  WireSlave.onReceive(receiveEvent);
  while (true) {
    WireSlave.update();
    delay(1);  // let I2C and other ESP32 peripherals interrupts work
  }
}

void setup_i2c()
{
  xTaskCreatePinnedToCore(I2CTask, "I2C Task", 4096, NULL, 24, NULL, 0);
}
