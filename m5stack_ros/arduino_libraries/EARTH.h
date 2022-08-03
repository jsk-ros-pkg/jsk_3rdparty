// Reference
// https://msr-r.net/m5stickc-unit/

#include <m5stack_ros.h>

#if defined(M5STACK)
  // Not tested
  // int digital_pin = 26;
  // int analog_pin = 36;
#elif defined(M5STICK_C) || defined(M5STICK_C_PLUS)
  int digital_pin = 32;
  int analog_pin = 33;
#endif

// Whether the target is moist
// true: moisture. false: not moisture
bool moist = false;
// Moisture content
// 0: Most moist. 4095: least moist.
int moisture = 4095;

void setupEARTH() {
  pinMode(digital_pin, INPUT);
}

void measureEARTH() {
  moist = digitalRead(digital_pin);
  moist = !moist;
  moisture = analogRead(analog_pin);
}

void displayEARTH() {
  M5.Lcd.setCursor(0, 0);
  if (moist) {
    M5.Lcd.printf("The target is moist.    \n");
  }
  else {
    M5.Lcd.printf("The target is not moist.\n");
  }
  M5.Lcd.printf("Moisture: %04d\n", moisture);
}
