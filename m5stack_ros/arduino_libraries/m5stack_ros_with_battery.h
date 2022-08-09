#include "m5stack_ros.h"
#include <IP5306.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

std_msgs::UInt16 level_msg;
ros::Publisher level_pub("battery_level", &level_msg);
std_msgs::Bool charging_msg;
ros::Publisher charging_pub("is_charging", &charging_msg);

bool is_sleeping = false;
void stopCb( const std_msgs::Empty& stop_msg ){ is_sleeping = true; };
void startCb( const std_msgs::Empty& start_msg ){ is_sleeping = false; };
ros::Subscriber<std_msgs::Empty> stop_sub("stop", &stopCb);
ros::Subscriber<std_msgs::Empty> start_sub("start", &startCb);

void setupBatteryPublisher(bool read_battery = true) {
  if (read_battery) {
    setupIP5306();
  }
  nh.advertise(level_pub);
  nh.advertise(charging_pub);
}

void setupSleepSubscriber() {
  nh.subscribe(stop_sub);
  nh.subscribe(start_sub);
}

void publishBattery(bool read_battery = true) {
  if (read_battery) {
    measureIP5306();
  }
  level_msg.data = battery_level;
  charging_msg.data = isCharging;
  level_pub.publish(&level_msg);
  charging_pub.publish(&charging_msg);
  nh.spinOnce();
}

void fillChargeIcon() {
  // 背景を真っ黒に塗りつぶす
  M5.Lcd.fillScreen(BLACK);
  // 電池へそ
  M5.Lcd.fillRoundRect(280,45,25,110,5,WHITE);
  // 大外枠
  M5.Lcd.fillRoundRect(40,20,240,160,5,WHITE);
  // 中枠
  int battery_bar = (int) (220 * battery_level / 100.0);
  M5.Lcd.fillRoundRect(50,30,battery_bar,140,5,GREEN);
  // 充電マーク
  M5.Lcd.fillTriangle(160, 65, 140, 110, 160, 110, BLACK);
  M5.Lcd.fillTriangle(160, 135, 180, 90, 160, 90, BLACK);
}

void blinkChargeIcon(bool read_battery = true) {
  if (read_battery) {
    measureIP5306();
  }
  fillChargeIcon();
  // 点滅
  delay(3000);
  M5.Lcd.sleep();
  M5.Lcd.setBrightness(0);
  delay(3000);
  M5.Lcd.wakeup();
  M5.Lcd.setBrightness(255);
}

void lightChargeIcon(bool read_battery = true) {
  // Currently, SerialClient connection timeout is 5*3=15 seconds
  // https://github.com/sktometometo/rosserial/blob/7b1bdfe6ee7fd14c0a0685cdbc674275f3aeb53b/rosserial_python/src/rosserial_python/SerialClient.py#L459
  // To avoid "Lost sync with device" error, check battery and update monitor every 10 (< 15) seconds
  if (read_battery) {
    measureIP5306();
  }
  fillChargeIcon();
  delay(10 * 1000);
  M5.Lcd.setBrightness(255);
}

// This function is additional setup process for m5stack_ros.
// this should be called after setup()
void setupCharge() {
  setupBatteryPublisher();
  setupSleepSubscriber();
}

// This function is additional loop process for m5stack_ros.
// This function should be called after every loop()
void checkCharge(bool read_battery = true) {
  // Do not enter main loop when is_sleeping is true
  while (is_sleeping || isCharging) {
    if (is_sleeping) {
      PRINTLN("Sleeping ...");
      delay(1000);
    }
    if (isCharging) {
      PRINTLN("Charging ...")
      M5.Lcd.setBrightness(255);
      // If we want to blink charge icon, use blinkChargeIcon()
      // blinkChargeIcon(read_battery);
      lightChargeIcon(read_battery);
    }
    nh.spinOnce();
    publishBattery(read_battery);
  }
  publishBattery(read_battery);
}
