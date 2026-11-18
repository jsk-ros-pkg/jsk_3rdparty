#include <DUAL_BUTTON.h>
#include <m5stack_ros.h>
#include <sensor_msgs/Joy.h>

sensor_msgs::Joy joy_msg;
ros::Publisher joy_pub("joy", &joy_msg);

void setup() {
  setupM5stackROS("M5Stack ROS DUAL BUTTON");
  setupDualButton(36,26);
  nh.advertise(joy_pub);
}

void loop() {

  // Read DualButton
  readDualButton();
  displayDualButton();

  // Publish ROS topic
  joy_msg.header.stamp = nh.now();
  joy_msg.buttons = buttons;
  joy_msg.buttons_length = 2;
  joy_pub.publish(&joy_msg);
  nh.spinOnce();

  delay(10);
}
