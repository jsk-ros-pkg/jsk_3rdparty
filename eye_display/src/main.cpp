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

EyeManagerIO eye = EyeManagerIO();

#if defined(USE_ROS) // defien callback
//ros::Subscriber<geometry_msgs::Point> sub_point("~look_at", [&](const geometry_msgs::Point &msg){eye.set_gaze_direction((float)msg.x, (float)msg.y);});
//ros::Subscriber<std_msgs::String> sub_eye_status("~eye_status", [&](const std_msgs::String &msg){eye.set_emotion(msg.data);});
#endif
#if !defined(USE_I2C) && !defined(USE_ROS) // sample code for eye asset without ROS/I2C
void setup_asset(EyeManager& eye)  // returns initial status
{
  std::map<std::string, EyeAsset>& eye_asset_map = eye.eye_asset_map;
  eye_asset_map["normal"] = EyeAsset();
  eye_asset_map["normal"].name = "normal";
  eye_asset_map["normal"].upperlid_position = {9};
  eye_asset_map["normal"].direction = 1;
  eye_asset_map["normal"].direction = true;

  eye_asset_map["blink"] = EyeAsset();
  eye_asset_map["blink"].name = "blink";
  eye_asset_map["blink"].upperlid_position = {9, 9, 130, 130, 9, 9};
  eye_asset_map["blink"].direction = 1;
  eye_asset_map["blink"].direction = true;

  eye_asset_map["happy"] = EyeAsset();
  eye_asset_map["happy"].name = "happy";
  eye_asset_map["happy"].path_iris = "/white.jpg";
  eye_asset_map["happy"].path_pupil = "/white.jpg";
  eye_asset_map["happy"].path_reflex = "/white.jpg";
  eye_asset_map["happy"].path_upperlid = "/reflex_happy.jpg";
  eye_asset_map["happy"].upperlid_position = {130, 131, 132, 133, 134, 135};
  eye_asset_map["happy"].direction = 1;
  eye_asset_map["happy"].direction = true;

  eye.set_emotion("happy");
}
#endif

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
#endif

  // initialize eye_asset_map and set current_eye_status
  setup_asset(eye);
  // draw eye image
  eye.update_look();

#if defined(USE_I2C)
  setup_i2c();
#endif
}

unsigned long last_time = 0;
unsigned long interval_time = 150;  // this will reproduce delay(100)
void loop()
{
#if defined(USE_ROS)  // USE_ROS
  reconnect_ros(eye);
#endif
  if ( last_time == 0 || ( millis() - last_time ) > interval_time*10 ) {
    last_time = millis();
    nh.loginfo("Reset timestamp");
  }
  long sleep_time = (last_time + interval_time) - millis();
  if ( sleep_time > 0 ) {
    delay(sleep_time);
  } else {
    sleep_time = 0;
  }
  unsigned long current_time = millis();
  if (current_time - last_time >= interval_time) {  // no drift
    last_time += interval_time;
  }

  // update emotion, this calls update_look to display
  int frame = eye.update_emotion();

#if defined(USE_ROS)
  nh.spinOnce();
  nh.loginfo("[%8ld] Eye status: %s (%d) (sleep %ld ms)", millis(), eye.get_emotion().c_str(), frame, sleep_time);
#endif

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
