// See
// https://qiita.com/ntrlmt/items/b7794f5f0705e1971b72

// If you use M5Stack, define M5STACK
// If you use M5StickC, define M5STICK_C
// If you use M5StickC, define M5STICK_C_PLUS
#if !defined(M5STACK) && !defined(M5STICK_C) && !defined(M5STICK_C_PLUS)
  #define M5STACK
#endif

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
/* #define ROSSERIAL_ARDUINO_TCP */
/* #define ROSSERIAL_ARDUINO_BLUETOOTH */

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
// There is also a size limit due to rosserial protocols. The message length is expressed in 2 bytes.
// https://wiki.ros.org/rosserial/Overview/Protocol#Packet_Format
// For melodic or older, the max message length is 32767 because message lengths are expressed in signed short.
// https://github.com/ros-drivers/rosserial/blob/5ff397fc17b6935ccf90ed840bcb874bfd608fe9/rosserial_python/src/rosserial_python/SerialClient.py#L501-L502
// For noetic, the max message length is 65535 because message lengths are expressed in unsigned short.
// https://github.com/ros-drivers/rosserial/blob/c169ae2173dcfda7cee567d64beae45198459400/rosserial_python/src/rosserial_python/SerialClient.py#L506-L507
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
    Serial.println("Start m5stack_ros with WiFi connection");
  #elif defined(ROSSERIAL_ARDUINO_BLUETOOTH)
    nh.initNode(name);
    SerialBT.begin(name);
    Serial.println("Start m5stack_ros with Bluetooth connection");
  #else
    nh.getHardware()->setBaud(57600);
    nh.initNode();
    Serial.println("Start m5stack_ros with USB connection");
  #endif
}

void setupM5stackROS() {
    setupM5stackROS("ROSSERIAL_BLUETOOTH");
}
