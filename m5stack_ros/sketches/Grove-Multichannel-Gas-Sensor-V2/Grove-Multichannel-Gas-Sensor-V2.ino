#include <Grove-Multichannel-Gas-Sensor-V2.h>
#include <m5stack_ros.h>
#include <std_msgs/UInt16.h>

std_msgs::UInt16 gas_v2_102b_msg;
std_msgs::UInt16 gas_v2_302b_msg;
std_msgs::UInt16 gas_v2_502b_msg;
std_msgs::UInt16 gas_v2_702b_msg;
ros::Publisher gas_v2_102b_pub("gas_v2_102b", &gas_v2_102b_msg);
ros::Publisher gas_v2_302b_pub("gas_v2_302b", &gas_v2_302b_msg);
ros::Publisher gas_v2_502b_pub("gas_v2_502b", &gas_v2_502b_msg);
ros::Publisher gas_v2_702b_pub("gas_v2_702b", &gas_v2_702b_msg);

void setup() {
  setupM5stackROS("M5Stack ROS Gas V2");
  setupGasV2();

  nh.advertise(gas_v2_102b_pub);
  nh.advertise(gas_v2_302b_pub);
  nh.advertise(gas_v2_502b_pub);
  nh.advertise(gas_v2_702b_pub);
}

void loop() {
  measureGasV2();
  displayGasV2();
    
  gas_v2_102b_msg.data = val_102B;
  gas_v2_302b_msg.data = val_302B;
  gas_v2_502b_msg.data = val_502B;
  gas_v2_702b_msg.data = val_702B;
  gas_v2_102b_pub.publish(&gas_v2_102b_msg);
  gas_v2_302b_pub.publish(&gas_v2_302b_msg);
  gas_v2_502b_pub.publish(&gas_v2_502b_msg);
  gas_v2_702b_pub.publish(&gas_v2_702b_msg);
  nh.spinOnce();

  delay(1000); //2000
}
