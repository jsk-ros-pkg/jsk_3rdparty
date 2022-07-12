#include <m5stack_ros_with_battery.h>
#include <PDM_SPM1423.h>
#include <audio_common_msgs/AudioData.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>

int loop_count = 1000;

audio_common_msgs::AudioData audio_msg;
ros::Publisher audio_pub("pdm_audio", &audio_msg);
std_msgs::Float32 volume_msg;
ros::Publisher volume_pub("pdm_volume", &volume_msg);

void pubBattery() {
  // Enable I2C
  measureIP5306();
  level_msg.data = battery_level;
  charging_msg.data = isCharging;
  level_pub.publish(&level_msg);
  charging_pub.publish(&charging_msg);
}

void pubAudio() {
  readMic();
  calcVolume();
  audio_msg.data = microRawData;
  audio_msg.data_length = bytesread;
  audio_pub.publish(&audio_msg);
  volume_msg.data = volume;
  volume_pub.publish(&volume_msg);
  // Draw volume on Lcd
  drawVolume(volume);
}

void setup() {
  setupM5stackROS();
  setupIP5306();
  microPhoneSetup();
  header("PDM Unit", BLACK);
  nh.advertise(audio_pub);
  nh.advertise(volume_pub);
  enableI2C();
  measureIP5306();
  afterSetup();
  disableI2C();
}

void loop() {
  // audio topic is published at about 35Hz, so battery topic is published once in about 30sec
  if (loop_count == 1000) {
    // Publish battery info while stopping publishing audio info
    // This is because I2C and I2S share the pin and they can't be measured simultaneously
    enableI2C();
    checkCharge();
    disableI2C();
    loop_count = 0;
    // Remove noisy audio data right after I2S starts
    delay(200);
  }
  else {
    pubAudio();
  }
  nh.spinOnce();
  loop_count++;
  delay(30);
}
