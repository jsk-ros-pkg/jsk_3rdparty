// Copied from
// https://github.com/m5stack/M5Unit-ENV/blob/eaa1983195c034b98b03918e4dd3521a894f9daa/examples/Unit_ENVIII_M5Core/Unit_ENVIII_M5Core.ino
//
// Required libraries are
// - M5Unit-ENV
//   https://github.com/m5stack/M5Unit-ENV
// - Adafruit_BMP280 version 2.6.3
//   https://github.com/adafruit/Adafruit_BMP280_Library

#include <m5stack_ros_with_battery.h>
#include <std_msgs/Float32.h>
#include <M5_ENV_III.h>

std_msgs::Float32 tmp_msg;
ros::Publisher tmp_pub("temperature", &tmp_msg);
std_msgs::Float32 hum_msg;
ros::Publisher hum_pub("humidity", &hum_msg);
std_msgs::Float32 pressure_msg;
ros::Publisher pressure_pub("pressure", &pressure_msg);

void publishENV() {
  tmp_msg.data = tmp;
  hum_msg.data = hum;
  pressure_msg.data = pressure;
  tmp_pub.publish(&tmp_msg);
  hum_pub.publish(&hum_msg);
  pressure_pub.publish(&pressure_msg);
}

void setup() {
  setupM5stackROS("M5Stack ROS ENVIII");
  setupENV();
  nh.advertise(tmp_pub);
  nh.advertise(hum_pub);
  nh.advertise(pressure_pub);
  setupCharge();
}

void loop() {
  checkCharge();
  measureENV();
  publishENV();
  nh.spinOnce();
  delay(10);
}
