#include <AS-MLV-P2_Cal_Gas_Sensor.h>
#include <m5stack_ros.h>
#include <std_msgs/UInt16.h>

std_msgs::UInt16 cal_gas_msg;
ros::Publisher cal_gas_pub("cal_gas", &cal_gas_msg);

void setup()
{
  setupM5stackROS("M5Stack ROS AS-MLV-P2 Cal Gas");
  setupCalGas();

  nh.advertise(cal_gas_pub);
}
void loop()
{
  measureCalGas();

  displayCalGas();

  cal_gas_msg.data = analog_value;
  cal_gas_pub.publish(&cal_gas_msg);
  nh.spinOnce();

  delay(200); //500
}
