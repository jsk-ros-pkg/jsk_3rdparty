// See
// https://qiita.com/Nabeshin/items/9268dc88927123319549

#include <m5stack_ros_with_battery.h>
#include <sensor_msgs/CompressedImage.h>
#include <jsk_recognition_msgs/Rect.h>
#include <jsk_recognition_msgs/RectArray.h>
#include <jsk_recognition_msgs/ClassificationResult.h>

// UART jpg communication
typedef struct {
    uint32_t  length; // Jpeg data size
    uint8_t   *buf;   // Jpeg data buffer
} jpeg_data_t;

jpeg_data_t jpeg_data;
#define DATA_SIZE 64 // data size except image data

#define DRAW_ON_LCD
#ifdef DRAW_ON_LCD
  // See https://qiita.com/Nabeshin/items/9268dc88927123319549
  #define LGFX_M5STACK
  // LovyanGFX version 0.4.1 is used
  #include <LovyanGFX.hpp>
  static LGFX lcd;
#endif

// Byte array to detect packet header
static const uint8_t packet_header[4] = { 0xFF, 0xD8, 0xEA, 0x01 };
// Byte array to stop/start UnitV
static const uint8_t packet_stop[4] = { 0xFF, 0xD8, 0xEA, 0x02 };
static const uint8_t packet_start[4] = { 0xFF, 0xD8, 0xEA, 0x03 };

sensor_msgs::CompressedImage unitv_img_msg;
ros::Publisher unitv_img_pub("unitv_image/compressed", &unitv_img_msg);
jsk_recognition_msgs::RectArray unitv_rects_msg;
ros::Publisher unitv_rects_pub("unitv_image/rects", &unitv_rects_msg);
jsk_recognition_msgs::ClassificationResult unitv_class_msg;
ros::Publisher unitv_class_pub("unitv_image/class", &unitv_class_msg);

char class_str[17];
int rects[4] = {0, 0, 0, 0};
int proba = 0;

int loop_count = 100;

void receive_recog_image() {
  uint8_t rx_buffer[DATA_SIZE];
  // Read DATA_SIZE byte
  int rx_size = Serial2.readBytes(rx_buffer, DATA_SIZE);
  if (! (rx_size == DATA_SIZE) ) {
    return;
  }
  // Specify packet header byte array
  if ((rx_buffer[0] == packet_header[0]) &&
      (rx_buffer[1] == packet_header[1]) &&
      (rx_buffer[2] == packet_header[2]) &&
      (rx_buffer[3] == packet_header[3])) {
      Serial.println("Packet header detected.");
      // Clear buffer
      while (Serial2.available() > 0) {
        Serial2.read();
      }
  }
  else{
    return;
  }
  // Read image and draw it on lcd
  jpeg_data.length = (uint32_t)(rx_buffer[41] << 16) | (rx_buffer[42] << 8) | rx_buffer[43];
  rx_size = Serial2.readBytes(jpeg_data.buf, jpeg_data.length);

  // Get recognition result
  char* data_str = (char *)rx_buffer+5;
  // Get class data

  int char_num = 0;
  while (!isspace(*(data_str+char_num))) {
    char_num++;
  }
  strncpy(class_str, data_str, char_num);
  class_str[char_num] = '\x0';
  Serial.print("Found object: ");
  Serial.println(class_str);
  // Get rect data
  char rect_str[5];
  for (int i=0; i<4; i++) {
    rects[i] = 0;
  }
  for (int i=0; i<4; i++) {
    strncpy(rect_str, data_str+16+i*4, 4);
    for (int j=0; j<4; j++) {
      rects[i] += (rect_str[j] - '0') * pow(10, 3-j);
    }
    if (rects[i] < 0)
      rects[i] = 0;
    rect_str[4] = '\x0';
  }
  // Get probability data
  char proba_str[5];
  proba = 0;
  strncpy(proba_str, data_str+32, 4);
  for (int j=0; j<4; j++) {
    proba += (proba_str[j] - '0') * pow(10, 3-j);
  }
  if (proba < 0)
    proba = 0;
  proba_str[4] = '\x0';
  Serial.println();
}

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

// Send packet to temporary disable UnitV UART
void write_packet_header() {
  Serial2.write(packet_header, 4);
}

// Send packet to stop UnitV
void write_packet_stop() {
  Serial2.write(packet_stop, 4);
}

// Send packet to start UnitV
// This is effective only when UnitV is already stopped
void write_packet_start() {
  Serial2.write(packet_start, 4);
}

void enableI2C() {
  write_packet_header();
  delay(1000);
  Serial2.end();
  Wire.begin();
}

void disableI2C() {
  Wire.endTransmission(true);
  Serial2.begin(115200, SERIAL_8N1, 21, 22);
}

void wait_for_unitv_image() {
  Serial.println("Wait for UnitV image to come.");
  while (!Serial2.available()) {
    delay(10);
  }
  receive_recog_image();
  Serial.println("UnitV image has come.");
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
  Serial.println("Wait for UnitV to wake up");
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
        Serial.println("Stop UnitV to save battery");
      }
      checkCharge(false);
      write_packet_start();
      Serial.println("Restart UnitV");
      loop_count = 0;
    }
    else {
      pub_unitv_image();
    }
    loop_count++;
    Serial.print("loop_count: ");
    Serial.println(loop_count);
    delay(500); // UnitV sends recognition result every 500ms
  }
  else {
    Serial.println("No new data has come in Serial2 (UART)");
    delay(50);
  }

  nh.spinOnce();
}
