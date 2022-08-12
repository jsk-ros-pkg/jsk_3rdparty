// Define NH_OUTPUT_SIZE before including m5stack_ros
// Increase NH_OUTPUT_SIZE to publish large size image
#define NH_OUTPUT_SIZE 32768
#include <m5stack_ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Float32.h>
#include <TimerCam.h>

sensor_msgs::CompressedImage timer_cam_img_msg;
ros::Publisher timer_cam_img_pub("timer_cam_image/compressed", &timer_cam_img_msg);
std_msgs::Float32 level_msg;
ros::Publisher level_pub("battery_level", &level_msg);

void publishTimerCam() {
  // Publish ROS image before releasing resources
  ros::Time time_now = nh.now();
  timer_cam_img_msg.header.stamp = time_now;
  timer_cam_img_msg.header.frame_id = "timer_cam";
  timer_cam_img_msg.format = "jpeg";
  timer_cam_img_msg.data_length = fb->len;
  timer_cam_img_msg.data = fb->buf;
  timer_cam_img_pub.publish( &timer_cam_img_msg );
}

void publishBattery() {
  level_msg.data = (float)bat_voltage / 1000.0;
  level_pub.publish( &level_msg );
}

void setup() {
  setupM5stackROS("M5Stack ROS TimerCam");
  setupTimerCam();
  setupBattery();

  // Wait for rosserial node
  delay(3000);
  nh.advertise(timer_cam_img_pub);
  nh.advertise(level_pub);
}

void loop() {
  // Update connection
  nh.spinOnce();
  delay(3000);

  // Publish before rosserial timeout (15 seconds)
  readTimerCam();
  readBattery();
  publishTimerCam();
  publishBattery();
  nh.spinOnce();

  // Wait for topics to be published
  delay(3000);

  // Deep sleep for 1 hour and then restart
  esp_deep_sleep((uint64_t)1 * 60 * 60 * 1000 * 1000);
}
