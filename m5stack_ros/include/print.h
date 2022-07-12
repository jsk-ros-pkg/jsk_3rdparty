#ifndef PRINT_H_INCLUDED
#define PRINT_H_INCLUDED

  // If bluetooth is used, print to BluetoothSerial as well as HardwareSerial
  #if defined(ROSSERIAL_ARDUINO_BLUETOOTH)
    #include <BluetoothSerial.h>
    BluetoothSerial SerialBT;
    #define PRINT(...) Serial.print(__VA_ARGS__);SerialBT.print(__VA_ARGS__);
    #define PRINTLN(...) Serial.println(__VA_ARGS__);SerialBT.println(__VA_ARGS__);
  #else
    #define PRINT(...) Serial.print(__VA_ARGS__);
    #define PRINTLN(...) Serial.println(__VA_ARGS__);
  #endif

#endif
