/*
    Description: The screen will display TVOC and CO2.
    Note: SGP30 needs 15 seconds to initialize calibration after power on.
*/

// Mainly copied from
// https://github.com/m5stack/M5-ProductExampleCodes/tree/master/Unit/TVOC/TVOC

#include "m5stack_ros_with_battery.h"
#include <std_msgs/UInt16.h>
#include <sound_play/SoundRequestActionGoal.h>
#include "Adafruit_SGP30.h"

std_msgs::UInt16 tvoc_msg;
ros::Publisher tvoc_pub("tvoc", &tvoc_msg);
std_msgs::UInt16 eco2_msg;
ros::Publisher eco2_pub("eco2", &eco2_msg);
sound_play::SoundRequestActionGoal sound_msg;
ros::Publisher sound_pub("sound_play/goal", &sound_msg);
long last_sound_play = 0;

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
  Serial.println("SGP30 test");
  if (! sgp.begin()){
    Serial.println("Sensor not found :(");
    while (1);
  }

  M5.Lcd.drawString("TVOC:", 50, 40, 4);
  M5.Lcd.drawString("eCO2:", 50, 80, 4);
  Serial.print("Found SGP30 serial #");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.println(sgp.serialnumber[2], HEX);
  M5.Lcd.drawString("Initialization...", 140, 120, 4);
}

void loopTVOCSGP30() {
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
    Serial.println("Measurement failed");
    return;
  }
  M5.Lcd.fillRect(100, 40, 220, 90, TFT_BLACK);
  M5.Lcd.drawNumber(sgp.TVOC, 120, 40 , 4);
  M5.Lcd.drawString("ppb", 200, 40, 4);
  M5.Lcd.drawNumber(sgp.eCO2, 120, 80, 4);
  M5.Lcd.drawString("ppm", 200, 80, 4);
  Serial.print("TVOC "); Serial.print(sgp.TVOC); Serial.print(" ppb\t");
  Serial.print("eCO2 "); Serial.print(sgp.eCO2); Serial.println(" ppm");
}

void setup() {
  setupM5stackROS();
  setupTVOCSGP30();
  nh.advertise(tvoc_pub);
  nh.advertise(eco2_pub);
  nh.advertise(sound_pub);

  afterSetup();
}

void loop() {
  beforeLoop();

  loopTVOCSGP30();

  tvoc_msg.data = sgp.TVOC;
  tvoc_pub.publish(&tvoc_msg);
  eco2_msg.data = sgp.eCO2;
  eco2_pub.publish(&eco2_msg);
  if ( millis() - last_sound_play > 30000 && sgp.eCO2 > 1000 ) {
    sound_msg.goal.sound_request.sound = -3;
    sound_msg.goal.sound_request.command = 1;
    sound_msg.goal.sound_request.volume = 1.0;
    sound_msg.goal.sound_request.arg = "C O 2 concentration is high. Please change the air.";
    sound_pub.publish(&sound_msg);
    last_sound_play = millis();
  }

  nh.spinOnce();
  delay(1000);
}
