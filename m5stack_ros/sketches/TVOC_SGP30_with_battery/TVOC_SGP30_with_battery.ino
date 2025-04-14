/*
    Description: The screen will display TVOC and CO2.
    Note: SGP30 needs 15 seconds to initialize calibration after power on.
*/

// Mainly copied from
// https://github.com/m5stack/M5-ProductExampleCodes/tree/master/Unit/TVOC/TVOC

#include "m5stack_ros_with_battery.h"
#include <TVOC_SGP30.h>
#include <std_msgs/UInt16.h>
#include <sound_play/SoundRequestActionGoal.h>

std_msgs::UInt16 tvoc_msg;
ros::Publisher tvoc_pub("tvoc", &tvoc_msg);
std_msgs::UInt16 eco2_msg;
ros::Publisher eco2_pub("eco2", &eco2_msg);
sound_play::SoundRequestActionGoal sound_msg;
ros::Publisher sound_pub("sound_play/goal", &sound_msg);
long last_sound_play = 0;

void setup() {
  setupM5stackROS("M5Stack ROS TVOC SGP30");
  setupTVOCSGP30();
  nh.advertise(tvoc_pub);
  nh.advertise(eco2_pub);
  nh.advertise(sound_pub);

  setupCharge();
}

void loop() {
  checkCharge();

  measureTVOCSGP30();

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
