#include <M5Stack.h>
#include <print.h>
#include "Adafruit_SGP30.h"

Adafruit_SGP30 sgp;
int i = 15;
long last_millis = 0;
void header(const char *string, uint16_t color)
{
    M5.Lcd.fillScreen(color);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Lcd.fillRect(0, 0, 320, 30, TFT_BLACK);
    M5.Lcd.setTextDatum(TC_DATUM);
    M5.Lcd.drawString(string, 160, 3, 4);
}

void setupTVOCSGP30() {
  header("SGP30 TEST",TFT_BLACK);
  PRINTLN("SGP30 test");
  if (! sgp.begin()){
    PRINTLN("Sensor not found :(");
    while (1);
  }

  M5.Lcd.drawString("TVOC:", 50, 40, 4);
  M5.Lcd.drawString("eCO2:", 50, 80, 4);
  PRINT("Found SGP30 serial #");
  PRINT(sgp.serialnumber[0], HEX);
  PRINT(sgp.serialnumber[1], HEX);
  PRINTLN(sgp.serialnumber[2], HEX);
  M5.Lcd.drawString("Initialization...", 140, 120, 4);
}

void measureTVOCSGP30() {
  while(i > 0) {
    if(millis()- last_millis > 1000) {
      last_millis = millis();
      i--;
      M5.Lcd.fillRect(198, 120, 40, 20, TFT_BLACK);
      M5.Lcd.drawNumber(i, 20, 120, 4);
    }
  }
  M5.Lcd.fillRect(0, 120, 300, 30, TFT_BLACK);

  if (! sgp.IAQmeasure()) {
    PRINTLN("Measurement failed");
    return;
  }
  M5.Lcd.fillRect(100, 40, 220, 90, TFT_BLACK);
  M5.Lcd.drawNumber(sgp.TVOC, 120, 40 , 4);
  M5.Lcd.drawString("ppb", 200, 40, 4);
  M5.Lcd.drawNumber(sgp.eCO2, 120, 80, 4);
  M5.Lcd.drawString("ppm", 200, 80, 4);
  PRINT("TVOC "); PRINT(sgp.TVOC); PRINTLN(" ppb");
  PRINT("eCO2 "); PRINT(sgp.eCO2); PRINTLN(" ppm");
  PRINTLN("");
}
