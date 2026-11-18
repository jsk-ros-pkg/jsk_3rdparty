// See
// https://qiita.com/Nabeshin/items/9268dc88927123319549
#include <M5Stack.h>
#include <print.h>

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
      PRINTLN("Packet header detected.");
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
  PRINT("Found object: ");
  PRINTLN(class_str);
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
  PRINTLN();
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
  PRINTLN("Wait for UnitV image to come.");
  while (!Serial2.available()) {
    delay(10);
  }
  receive_recog_image();
  PRINTLN("UnitV image has come.");
}
