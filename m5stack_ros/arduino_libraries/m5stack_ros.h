// See
// https://qiita.com/ntrlmt/items/b7794f5f0705e1971b72

// If you use M5Stack, define M5STACK
// If you use M5StickC, define M5STICK_C
// If you use M5StickC, define M5STICK_C_PLUS
#define M5STACK
/* #define ROSSERIAL_ARDUINO_TCP */
/* #define ROSSERIAL_ARDUINO_BLUETOOTH */

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

// If ROSSERIAL_ARDUINO_BLUETOOTH is defined,
// BluetoothSerial is used as well as Hardwareserial
#include <print.h>

// MAX_SUBSCRIBERS, MAX_PUBLISHERS, INPUT_SIZE, OUTPUT_SIZE
// If you change these parameters, define these macros before including this file
// Each parameter has limitation depending on the microcontroller chip
// https://wiki.ros.org/rosserial/Overview/Limitations
#ifndef NH_MAX_SUBSCRIBERS
 #define NH_MAX_SUBSCRIBERS 25
#endif
#ifndef NH_MAX_PUBLISHERS
  #define NH_MAX_PUBLISHERS 25
#endif
#ifndef NH_INPUT_SIZE
  #define NH_INPUT_SIZE 8192
#endif
#ifndef NH_OUTPUT_SIZE
  #define NH_OUTPUT_SIZE 8192
#endif
ros::NodeHandle_<ArduinoHardware,
                 NH_MAX_SUBSCRIBERS,
                 NH_MAX_PUBLISHERS,
                 NH_INPUT_SIZE,
                 NH_OUTPUT_SIZE> nh;

void setupM5stackROS(char *name) {
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
    nh.initNode();
  #elif defined(ROSSERIAL_ARDUINO_BLUETOOTH)
    nh.initNode(name);
    SerialBT.begin(name);
  #else
    nh.getHardware()->setBaud(57600);
    nh.initNode();
  #endif
}

void setupM5stackROS() {
    setupM5stackROS("ROSSERIAL_BLUETOOTH");
}
