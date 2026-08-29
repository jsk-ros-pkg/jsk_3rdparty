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
  nh.advertise(timer_cam_img_pub);
  nh.advertise(level_pub);
  setupTimerCam();
  setupBattery();
}

void loop() {
  readTimerCam();
  readBattery();
  publishTimerCam();
  publishBattery();
  nh.spinOnce();
  // High fps setting will cause CompressedImage topics
  // to continue to accumulate in publisher queue and increase latency.
  float fps = 1.0;
  delay(1000.0 / fps);
}
