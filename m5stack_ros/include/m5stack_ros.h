// See
// https://qiita.com/ntrlmt/items/b7794f5f0705e1971b72

// If you use M5Stack, define M5STACK
// If you use M5StickC, define M5STICK_C
// If you use M5StickC, define M5STICK_C_PLUS
#define M5STACK

#if defined(M5STACK)
  #include <M5Stack.h>
#elif defined(M5STICK_C)
  #include <M5StickC.h>
#elif defined(M5STICK_C_PLUS)
  #include <M5StickCPlus.h>
#endif

#include <esp_info.h>

// Define connection type.
// If you use Bluetooth, define ROSSERIAL_ARDUINO_BLUETOOTH
// If you use Wi-Fi, define ROSSERIAL_ARDUINO_TCP
// If you use USB, do not define anything
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
  #if defined(M5STACK)
    M5.Speaker.begin();
    M5.Speaker.mute();
  #endif
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
