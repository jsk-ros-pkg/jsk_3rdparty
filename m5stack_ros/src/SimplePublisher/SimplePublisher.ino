// See
// https://qiita.com/ntrlmt/items/b7794f5f0705e1971b72

// Include ROS files
#include <m5stack_ros.h>
#include <std_msgs/String.h>

// ROS msg and ROS publisher
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

void setup()
{
  // Setup m5stack for ROS
  setupM5stackROS();
  nh.advertise(chatter);

  // Visualization on m5stack monitor
  M5.Lcd.printf("ROS Hello World Publisher Started\n");
}

void loop()
{
  char hello[13] = "hello world!";
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
