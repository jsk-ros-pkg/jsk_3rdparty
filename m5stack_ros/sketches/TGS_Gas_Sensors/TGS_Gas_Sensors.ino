#include <TGS_Gas_Sensors.h>
#include <m5stack_ros.h>
#include <std_msgs/UInt16.h>

std_msgs::UInt16 tgs_gas_analog_msg;
std_msgs::UInt16 tgs_gas_digital_msg;
ros::Publisher tgs_gas_analog_pub("tgs_gas_analog", &tgs_gas_analog_msg);
ros::Publisher tgs_gas_digital_pub("tgs_gas_digital", &tgs_gas_digital_msg);

void setup()
{
  setupM5stackROS("M5Stack ROS TGS_Gas_Sensor");
  setupTGSSensors();

  nh.advertise(tgs_gas_analog_pub);
  nh.advertise(tgs_gas_digital_pub);
}
void loop()
{
  measureTGSSensors();

  displayTGSSensors();

  tgs_gas_analog_msg.data = analog_value;
  tgs_gas_digital_msg.data = digital_value;
  tgs_gas_analog_pub.publish(&tgs_gas_analog_msg);
  tgs_gas_digital_pub.publish(&tgs_gas_digital_msg);
  nh.spinOnce();
  
  delay(200);
}
