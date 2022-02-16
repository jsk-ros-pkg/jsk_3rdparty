#include <m5stack_ros.h>

// I2S microphone
#include <PDM_SPM1423.h>
#include <audio_common_msgs/AudioData.h>
#include <std_msgs/Float32.h>
// I2C battery management module
#include <IP5306.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>

audio_common_msgs::AudioData audio_msg;
ros::Publisher audio_pub("audio", &audio_msg);
std_msgs::Float32 volume_msg;
ros::Publisher volume_pub("volume", &volume_msg);

std_msgs::UInt16 level_msg;
ros::Publisher level_pub("battery_level", &level_msg);
std_msgs::Bool charging_msg;
ros::Publisher charging_pub("is_charging", &charging_msg);

int loop_count = 0;

void pubAudio() {
  // Read microphone data
  readMic();
  calcVolume();
  // Publish ROS topics
  audio_msg.data = microRawData;
  audio_msg.data_length = bytesread;
  audio_pub.publish(&audio_msg);
  volume_msg.data = volume;
  volume_pub.publish(&volume_msg);
  // Draw volume on Lcd
  drawVolume(volume);
}

void pubBattery() {
  measureIP5306();
  level_msg.data = battery_level;
  charging_msg.data = isCharging;
  level_pub.publish(&level_msg);
  charging_pub.publish(&charging_msg);
  nh.spinOnce();
}

void setup() {
  setupM5stackROS();
  microPhoneSetup();
  enableI2C();
  setupIP5306();
  disableI2C();
  header("PDM Unit", BLACK);
  nh.advertise(audio_pub);
  nh.advertise(volume_pub);
  nh.advertise(level_pub);
  nh.advertise(charging_pub);
}

void loop() {
  // audio topic is published at about 35Hz, so battery topic is published once in about 30sec
  if (loop_count == 1000) {
    // Publish battery info while stopping publishing audio info
    // This is because I2C and I2S share the pin and they can't be measured simultaneously
    enableI2C();
    pubBattery();
    disableI2C();
    loop_count = 0;
    // Remove noisy audio data right after I2S starts
    delay(200);
  }
  pubAudio();
  nh.spinOnce();
  loop_count++;
}
