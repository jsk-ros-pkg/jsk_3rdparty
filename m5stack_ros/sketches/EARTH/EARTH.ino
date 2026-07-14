#include <EARTH.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>

std_msgs::Bool moist_msg;
ros::Publisher moist_pub("moist", &moist_msg);
std_msgs::Int16 moisture_msg;
ros::Publisher moisture_pub("moisture", &moisture_msg);

void setup()
{
  setupM5stackROS("M5Stack ROS EARTH");
  setupEARTH();

  nh.advertise(moist_pub);
  nh.advertise(moisture_pub);
}

void loop()
{
  measureEARTH();

  displayEARTH();

  moist_msg.data = moist;
  moisture_msg.data = moisture;
  moist_pub.publish(&moist_msg);
  moisture_pub.publish(&moisture_msg);
  nh.spinOnce();

  delay(1000);
}
