// See
// https://qiita.com/ntrlmt/items/b7794f5f0705e1971b72

// Define connection type.
// If you use Bluetooth, define ROSSERIAL_ARDUINO_BLUETOOTH
// If you use Wi-Fi, define ROSSERIAL_ARDUINO_TCP
// If you use USB, do not define anything

#include <M5Stack.h>
#include <esp_info.h>

#if defined(ROSSERIAL_ARDUINO_TCP)
  #include <ros.h>
  #include <wifi.h>
// ESP_SERIAL is to avoid tcp in melodic
// https://github.com/ros-drivers/rosserial/pull/448
// https://github.com/ros-drivers/rosserial/pull/559
#elif defined(ROSSERIAL_ARDUINO_BLUETOOTH)
  #define ESP_SERIAL
  #include <ros.h>
#else
  #define ESP_SERIAL
  #include <ros.h>
#endif

// MAX_SUBSCRIBERS, MAX_PUBLISHERS, INPUT_SIZE, OUTPUT_SIZE
ros::NodeHandle_<ArduinoHardware, 25, 25, 8192, 8192> nh;

void setupM5stackROS() {
  M5.begin();
  M5.Speaker.begin();
  M5.Speaker.mute();
  Serial.begin(115200);
  esp_info();

  #if defined(ROSSERIAL_ARDUINO_TCP)
    setupWiFi();
    nh.getHardware()->setConnection(server);
  #elif defined(ROSSERIAL_ARDUINO_BLUETOOTH)
  #else
    nh.getHardware()->setBaud(57600);
  #endif

  nh.initNode();
}
