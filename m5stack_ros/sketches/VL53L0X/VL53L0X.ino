#include <VL53L0X_.h>
#include <m5stack_ros.h>
#include <std_msgs/UInt16.h>

std_msgs::UInt16 tof_msg;
ros::Publisher tof_pub("tof", &tof_msg);

void setup()
{
  setupM5stackROS("M5Stack ROS VL53L0X");
  setupVL53L0X();

  nh.advertise(tof_pub);
}

void loop()
{
  measureVL53L0X();

  displayVL53L0X();

  tof_msg.data = tof;
  tof_pub.publish(&tof_msg);
  nh.spinOnce();

  delay(100);
}
