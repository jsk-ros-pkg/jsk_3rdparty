#include <m5stack_ros_with_battery.h>
#include <yolov2.h>
#include <sensor_msgs/CompressedImage.h>
#include <jsk_recognition_msgs/Rect.h>
#include <jsk_recognition_msgs/RectArray.h>
#include <jsk_recognition_msgs/ClassificationResult.h>

sensor_msgs::CompressedImage unitv_img_msg;
ros::Publisher unitv_img_pub("unitv_image/compressed", &unitv_img_msg);
jsk_recognition_msgs::RectArray unitv_rects_msg;
ros::Publisher unitv_rects_pub("unitv_image/rects", &unitv_rects_msg);
jsk_recognition_msgs::ClassificationResult unitv_class_msg;
ros::Publisher unitv_class_pub("unitv_image/class", &unitv_class_msg);

int loop_count = 100;

void pub_unitv_image () {
  ros::Time time_now = nh.now();
  // img msg
  unitv_img_msg.header.stamp = time_now;
  unitv_img_msg.header.frame_id = "unitv";
  unitv_img_msg.format = "jpeg";
  unitv_img_msg.data_length = jpeg_data.length;
  unitv_img_msg.data = jpeg_data.buf;
  unitv_img_pub.publish( &unitv_img_msg );
  // rects msg
  unitv_rects_msg.header.stamp = time_now;
  unitv_rects_msg.header.frame_id = "unitv";
  jsk_recognition_msgs::Rect rect_msg[1];
  rect_msg[0].x = rects[0];
  rect_msg[0].y = rects[1];
  rect_msg[0].width = rects[2];
  rect_msg[0].height = rects[3];
  unitv_rects_msg.rects_length = 1;
  unitv_rects_msg.rects = rect_msg;
  unitv_rects_pub.publish( &unitv_rects_msg );
  // class msg
  unitv_class_msg.header.stamp = time_now;
  unitv_class_msg.header.frame_id = "unitv";
  unitv_class_msg.labels_length = 1;
  uint32_t labels[1] = {0};
  unitv_class_msg.labels = labels;
  unitv_class_msg.label_names_length = 1;
  char *label_names[1] = {class_str};
  unitv_class_msg.label_names = label_names;
  unitv_class_msg.label_proba_length = 1;
  float probas[1];
  probas[0] = proba;
  unitv_class_msg.label_proba = probas;
  unitv_class_msg.classifier = "yolov2";
  unitv_class_msg.target_names_length = 1;
  char *target_names[1] = {"dummy"};
  unitv_class_msg.target_names = target_names;
  unitv_class_pub.publish( &unitv_class_msg );

  #ifdef DRAW_ON_LCD
    lcd.drawJpg(jpeg_data.buf, jpeg_data.length,0, 0, 320, 240, 0, 0, ::JPEG_DIV_NONE);
    // Camera image is scaled in UnitV
    // We need to calculate rectanble position considering the scale
    float scale = 0.6;
    M5.Lcd.drawRect(
      int(rects[0] / scale),
      int(rects[1] / scale),
      int(rects[2] / scale),
      int(rects[3] / scale),
      RED);
    int string_x = max(0, min(320, int(rects[0]/scale)));
    int string_y = max(0, min(320, int(rects[1]/scale)));
    M5.Lcd.drawString(
      class_str,
      string_x,
      string_y);
  #endif
}

void setup() {
  // Setup ROS except battery modules
  setupM5stackROS("M5Stack ROS UnitV yolov2");
  #ifdef DRAW_ON_LCD
    lcd.init();
    lcd.setRotation(1);
    lcd.setBrightness(255);
    lcd.setColorDepth(16);
    lcd.clear();
    M5.Lcd.setTextSize(3);
  #else
    M5.Lcd.setBrightness(0);
  #endif
  nh.advertise(unitv_img_pub);
  nh.advertise(unitv_rects_pub);
  nh.advertise(unitv_class_pub);

  // malloc size must be less than 8192 byte.
  // https://www.mgo-tec.com/blog-entry-trouble-shooting-esp32-wroom.html/5#title30
  // malloc size must be larger than the size of the JPEG image received from UnitV.
  jpeg_data.buf = (uint8_t *) malloc(sizeof(uint8_t) * 7000);

  // Wait for UnitV to initialize (to start accepting write_data() input)
  PRINTLN("Wait for UnitV to wake up");
  Serial2.begin(115200, SERIAL_8N1, 21, 22);
  wait_for_unitv_image();

  // Temporary disable UART and enable I2C. In the meantime, initialize battery module.
  enableI2C();
  M5.Power.begin();
  afterSetup();
  disableI2C();
}

void loop() {
  // Publish UnitV image
  if (Serial2.available()) {
    receive_recog_image();
    // For every 100 UnitV images published, battery info is published.
    // Note that UnitV image is published at about 1Hz
    if (loop_count == 100) {
      enableI2C();
      measureIP5306();
      disableI2C();
      if (is_sleeping || isCharging) {
        // Stop UnitV to save computation power and M5Stack battery
        wait_for_unitv_image();
        write_packet_stop();
        PRINTLN("Stop UnitV to save battery");
      }
      checkCharge(false);
      write_packet_start();
      PRINTLN("Restart UnitV");
      loop_count = 0;
    }
    else {
      pub_unitv_image();
    }
    loop_count++;
    PRINT("loop_count: ");
    PRINTLN(loop_count);
    delay(500); // UnitV sends recognition result every 500ms
  }
  else {
    PRINTLN("No new data has come in Serial2 (UART)");
    delay(50);
  }

  nh.spinOnce();
}
