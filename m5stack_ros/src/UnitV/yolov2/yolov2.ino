// See
// https://qiita.com/Nabeshin/items/9268dc88927123319549

#include <m5stack_ros.h>
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

// Byte array to detect packet header
static const uint8_t packet_header[4] = { 0xFF, 0xD8, 0xEA, 0x01 };

sensor_msgs::CompressedImage unitv_img_msg;
ros::Publisher unitv_img_pub("unitv_image/compressed", &unitv_img_msg);
jsk_recognition_msgs::RectArray unitv_rects_msg;
ros::Publisher unitv_rects_pub("unitv_image/rects", &unitv_rects_msg);
jsk_recognition_msgs::ClassificationResult unitv_class_msg;
ros::Publisher unitv_class_pub("unitv_image/class", &unitv_class_msg);

char class_str[17];
int rects[4] = {0, 0, 0, 0};
int proba = 0;

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
}

void setup() {
  m5stack_ros_setup();

  Serial2.begin(115200, SERIAL_8N1, 21, 22);
  jpeg_data.buf = (uint8_t *) malloc(sizeof(uint8_t) * 100000);

  nh.advertise(unitv_img_pub);
  nh.advertise(unitv_rects_pub);
  nh.advertise(unitv_class_pub);
}

void loop() {
  if (Serial2.available()) {
    receive_recog_image();
    pub_unitv_image();
    delay(500);
  }
  else {
    Serial.println("No new data has come in Serial2 (UART)");
  }
  nh.spinOnce();

  delay(10);
}
