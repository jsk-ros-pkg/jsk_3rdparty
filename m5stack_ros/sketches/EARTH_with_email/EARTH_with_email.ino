#include <EARTH.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

std_msgs::Bool moist_msg;
ros::Publisher moist_pub("moist", &moist_msg);
std_msgs::Int16 moisture_msg;
ros::Publisher moisture_pub("moisture", &moisture_msg);
std_msgs::Float32 battery_msg;
ros::Publisher battery_pub("battery_level", &battery_msg);
std_msgs::Bool low_bat_msg;
ros::Publisher low_bat_pub("low_battery", &low_bat_msg);

void publishEARTH()
{
  measureEARTH();
  moist_msg.data = moist;
  moisture_msg.data = moisture;
  moist_pub.publish(&moist_msg);
  moisture_pub.publish(&moisture_msg);
}

void publishBattery()
{
  battery_msg.data = M5.Axp.GetBatVoltage();
  low_bat_msg.data = M5.Axp.GetWarningLevel();
  battery_pub.publish(&battery_msg);
  low_bat_pub.publish(&low_bat_msg);
}

void blackScreen()
{
  // Black screen to save energy consumption
  #if defined(M5STACK)
    M5.Lcd.setBrightness(0);
  #elif defined(M5STICK_C) || defined(M5STICK_C_PLUS)
    M5.Axp.SetLDO2(false);
  #endif
}

void setup()
{
  setupM5stackROS("M5Stack ROS EARTH");
  setupEARTH();
  blackScreen();

  // Wait for rosserial node
  delay(3000);
  nh.advertise(moist_pub);
  nh.advertise(moisture_pub);
  nh.advertise(battery_pub);
  nh.advertise(low_bat_pub);
}

void loop()
{
  // Update connection
  nh.spinOnce();
  delay(3000);

  // Publish before rosserial timeout (15 seconds)
  publishEARTH();
  publishBattery();
  nh.spinOnce();
  // Wait for topics to be published
  delay(3000);

  // Deep sleep for 1 hour and then restart
  esp_deep_sleep((uint64_t)1 * 60 * 60 * 1000 * 1000);
}
