#include <m5stack_ros.h>
#include <PDM_SPM1423.h>
#include <audio_common_msgs/AudioData.h>
#include <std_msgs/Float32.h>

audio_common_msgs::AudioData audio_msg;
ros::Publisher audio_pub("audio", &audio_msg);
std_msgs::Float32 volume_msg;
ros::Publisher volume_pub("volume", &volume_msg);

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

void setup() {
  setupM5stackROS();
  microPhoneSetup();
  header("PDM Unit", BLACK);
  nh.advertise(audio_pub);
  nh.advertise(volume_pub);
}

void loop() {
  pubAudio();
  nh.spinOnce();
}
