#include <Arduino.h>

#include "eye.hpp"

#if defined(USE_I2C)
#include "i2c_lib.h"
#endif
#if defined(USE_ROS)
#include "ros_lib.h"
#endif

#if defined(STAMPS3)
#include "ArduinoHWCDCHardware.h"
#elif defined(STAMPC3)
#include "ArduinoHardware.h"
#endif

#define TFT_BL 10 // LED back-light control pin

const int image_width = 139;
const int image_height = 139;

EyeManager eye = EyeManager();

std::string eye_asset_text =
  "eye_asset_names: normal, blink, happy\n"
  "eye_asset_position: normal: 9\n"
  "eye_asset_position: blink: 9, 9, 130, 130, 9, 9\n"
  "eye_asset_image_path: happy: iris: /white.jpg\n"
  "eye_asset_image_path: happy: pupil: /white.jpg\n"
  "eye_asset_image_path: happy: reflex: /white.jpg\n"
  "eye_asset_image_path: happy: upperlid: /reflex_happy.jpg\n"
  "eye_asset_position: happy: 130, 131, 132, 133, 134, 135\n";


unsigned long interval_time = 150;  // this will reproduce delay(100)
unsigned long next_time = millis() + interval_time;

void setup()
{
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  SPIFFS.begin();
  Serial.begin(115200);

  // initialize eye manager with current eye asset
  eye.init();
  // draw eye image
  eye.update_look();

#if defined(USE_ROS)  // USE_ROS
  setup_ros();
  eye_asset_text = ros_read_asset();
#endif

  // initialize eye_asset_map and set current_eye_status
  eye.setup_asset(eye_asset_text);
  // draw eye image
  eye.update_look();

#if defined(USE_I2C)
  setup_i2c();
#endif

  next_time = millis() + interval_time;
  loginfo("[%8ld] setup() done: next_time = %ld", next_time);
}

void loop()
{
#if defined(USE_ROS)  // USE_ROS
  reconnect_ros(eye);
#endif
  long sleep_time = next_time - millis();
  if ( sleep_time > 0 ) {
    delay(sleep_time);
  }
  next_time += interval_time;

  // update emotion, this calls update_look to display
  int frame = eye.update_emotion();

#if defined(USE_ROS)
  nh.spinOnce();
#endif
  loginfo("[%8ld] Eye status: %s (%d) (sleep %ld ms)", millis(), eye.get_emotion().c_str(), frame, sleep_time);

#if !defined(USE_I2C) && !defined(USE_ROS) // sample code for eye asset
  static float look_x = 0;
  static float look_y = 0;
  look_x = 10.0 * sin(frame * 0.1);
  look_y = 10.0 * cos(frame * 0.1) ;
  eye.set_gaze_direction(look_x, look_y);

  if (frame % 10 == 0) {
    static auto eye_asset = eye.eye_asset_map.begin();
    eye_asset++;
    if (eye_asset == eye.eye_asset_map.end()) eye_asset = eye.eye_asset_map.begin();
    eye.set_emotion(eye_asset->first);
  }
#endif
}
