#include <IP5306.h>
#include <m5stack_ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>

std_msgs::UInt16 level_msg;
ros::Publisher level_pub("battery_level", &level_msg);
std_msgs::Bool charging_msg;
ros::Publisher charging_pub("is_charging", &charging_msg);

void setup()
{
  setupM5stackROS();
  setupIP5306();

  nh.advertise(level_pub);
  nh.advertise(charging_pub);
}

void loop() {
  measureIP5306();

  displayIP5306();

  level_msg.data = battery_level;
  charging_msg.data = isCharging;
  level_pub.publish(&level_msg);
  charging_pub.publish(&charging_msg);
  nh.spinOnce();

  delay(1000);
}
