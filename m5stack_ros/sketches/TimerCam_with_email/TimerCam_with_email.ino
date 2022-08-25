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

#define GROVE_CIRCULAR_LED
#ifdef GROVE_CIRCULAR_LED
  #include <Grove_Circular_LED.h>
#endif

// Use BMM8563_SLEEP for low energy consumption sleep
// Do not use BMM8563_SLEEP if you use normal delay
// For detail, see README
// #define BMM8563_SLEEP

int interval = 1 * 60 * 60; // seconds

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
  #ifdef GROVE_CIRCULAR_LED
    setupGroveCircularLED();
  #endif

  // Wait for rosserial node
  delay(3000);
  nh.advertise(timer_cam_img_pub);
  nh.advertise(level_pub);

  #ifdef BMM8563_SLEEP
    // For sleep and wakeup
    bmm8563_init();
    // interval [sec] later will wake up
    bmm8563_setTimerIRQ(interval);
  #endif
}

void loop() {
  // Update connection. Longer delay is needed for large size topic(?)
  // We need to publish topics before rosserial timeout (15 seconds) after the connection is created
  nh.spinOnce();
  delay(4000);
  nh.spinOnce();
  delay(4000);

  // Read image
  #ifdef GROVE_CIRCULAR_LED
    turnOnGroveCircularLED();
    delay(100);
  #endif
  // Read camera image several times and use the latest image. Early images are darker then normal.
  for(int i=0; i<3; i++) {
    readTimerCam();
    delay(100);
  }
  #ifdef GROVE_CIRCULAR_LED
    turnOffGroveCircularLED();
  #endif
  // Read battery data
  readBattery();
  Serial.println("Read sensor data");
  // nh.spinOnce() to update nh.now()
  nh.spinOnce();
  // Try to send topics for several times because sometimes package is lost
  for(int i=0; i<3; i++) {
    publishTimerCam();
    publishBattery();
    nh.spinOnce();
    Serial.println("Publish topics");
    delay(1000);
  }

  Serial.println("Sleep");
  #ifdef BMM8563_SLEEP
    // Disable bat output, will wake up after interval [sec], Sleep current is 1~2μA
    // Do not use deep sleep because esp32 cannot wakeup by itself without usb charging
    bat_disable_output();
    // if usb not connect, will not in here;
    esp_deep_sleep((uint64_t)(interval * 1000 * 1000));
    esp_deep_sleep_start();
  #else
    delay(interval * 1000);
  #endif
  Serial.println("Wakeup");
}
