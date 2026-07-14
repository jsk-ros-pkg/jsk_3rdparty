#include <VCNL4040.h>
#include <m5stack_ros.h>
#include <std_msgs/UInt16.h>

std_msgs::UInt16 proximity_msg;
ros::Publisher proximity_pub("proximity", &proximity_msg);

void setup()
{
  setupM5stackROS("M5Stack ROS VCNL4040");
  setupVCNL4040();

  nh.advertise(proximity_pub);
}

void loop() {
  measureVCNL4040();

  displayVCNL4040();

  proximity_msg.data = proximity;
  proximity_pub.publish(&proximity_msg);
  nh.spinOnce();

  delay(100);
}
