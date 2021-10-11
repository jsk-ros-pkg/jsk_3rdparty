// See
// https://programresource.net/2020/02/24/2960.html
// https://m5stack-build.hatenablog.com/entry/2019/10/10/222405

#include <M5Stack.h>

// R, B: 0~31
// G:    0~63
#define RGB(r,g,b) (int16_t)(b + (g << 5) + (r << 11))

int battery_level = 0;
bool isCharging = false;

void setupIP5306()
{
  M5.Power.begin();
  if(!M5.Power.canControl()) {
    //can't control.
    Serial.println("Cannot control M5.Power");
    return;
  }
  Wire.begin();
}

void measureIP5306()
{
  // Get battery level
  byte retval;
  Wire.beginTransmission(0x75);
  Wire.write(0x78);
  if (Wire.endTransmission(false) == 0 && Wire.requestFrom(0x75, 1)) {
    retval = Wire.read() & 0xF0;
    if (retval == 0xE0) battery_level = 25;
    else if (retval == 0xC0) battery_level = 50;
    else if (retval == 0x80) battery_level = 75;
    else if (retval == 0x00) battery_level = 100;
  }

  // is M5Stack Charging?
  isCharging = M5.Power.isCharging();
}

void displayIP5306()
{
  // M5.Lcd.clear();
  M5.Lcd.setTextSize(3);

  // Display battery level icon
  M5.Lcd.fillRect(250, 5, 56, 21, RGB(31, 63, 31));
  M5.Lcd.fillRect(306, 9, 4, 13, RGB(31, 63, 31));
  M5.Lcd.fillRect(252, 7, 52, 17, RGB(0, 0, 0));
  if (battery_level <= 25)
    M5.Lcd.fillRect(253, 8, battery_level/2, 15, RGB(31, 20, 10));
  else
    M5.Lcd.fillRect(253, 8, battery_level/2, 15, RGB(20, 40, 31));
  // Display charging icon
  if (true == isCharging) {
    M5.Lcd.fillTriangle(278, 8, 273, 17, 278, 17, WHITE);
    M5.Lcd.fillTriangle(278, 22, 283, 13, 278, 13, WHITE);
  }

  // Print battery level
  M5.Lcd.setCursor(0, 50);
  M5.Lcd.printf("Battery level\n\n %u %%\n\n\n", battery_level);
  if (true == isCharging)
    M5.Lcd.printf("Charging...    ");
  else
    M5.Lcd.printf("Not charging...");
}
