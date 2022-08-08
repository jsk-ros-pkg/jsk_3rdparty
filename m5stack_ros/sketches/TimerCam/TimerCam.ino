#include <m5stack_ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <TimerCam.h>

sensor_msgs::CompressedImage timer_cam_img_msg;
ros::Publisher timer_cam_img_pub("timer_cam_image/compressed", &timer_cam_img_msg);

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

void setup() {
  setupM5stackROS("M5Stack ROS TimerCam");
  nh.advertise(timer_cam_img_pub);
  setupTimerCam();
}

void loop() {
  readTimerCam();
  publishTimerCam();
  nh.spinOnce();
  float fps = 10.0;
  delay(1000.0 / fps);
}
