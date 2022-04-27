#include <JOYSTICK.h>
#include <m5stack_ros.h>
#include <sensor_msgs/Joy.h>

sensor_msgs::Joy joy_msg;
ros::Publisher joy_pub("joy", &joy_msg);

void setup() {
  setupM5stackROS();
  Wire.begin(21, 22, 400000UL);
  setupJoystick();
  nh.advertise(joy_pub);
}

char data[100];
void loop() {
  // Read Joystick
  readJoystick();
  displayJoystick();

  // Publish ROS topic
  joy_msg.header.stamp = nh.now();
  joy_msg.axes = axes;
  joy_msg.axes_length = 2;
  joy_msg.buttons = buttons;
  joy_msg.buttons_length = 1;
  joy_pub.publish(&joy_msg);
  nh.spinOnce();

  delay(10);
}
