// Mainly copied from
// https://github.com/m5stack/M5-ProductExampleCodes/tree/master/Unit/THERMAL/Arduino

#include <m5stack_ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int16.h>
#include "MLX90640.h"

#define DRAW_ON_LCD

std_msgs::Int16 min_msg;
ros::Publisher min_pub("thermal/min_temp", &min_msg);

std_msgs::Int16 max_msg;
ros::Publisher max_pub("thermal/max_temp", &max_msg);

std_msgs::Int16 center_msg;
ros::Publisher center_pub("thermal/center_temp", &center_msg);

sensor_msgs::Image img_msg;
ros::Publisher img_pub("thermal/image", &img_msg);
uint8_t img_buf[INTERPOLATED_COLS * INTERPOLATED_ROWS * 3];

void pubMinTemp()
{
  min_msg.data = min_v;
  min_pub.publish(&min_msg);
}

void pubMaxTemp()
{
  max_msg.data = max_v;
  max_pub.publish(&max_msg);
}

void pubCenterTemp()
{
  center_msg.data = spot_v;
  center_pub.publish(&center_msg);
}

void pubImage(float *p, uint8_t rows, uint8_t cols)
{
  int colorTemp;

  for (int y = 0; y < rows; y++)
  {
    for (int x = 0; x < cols; x++)
    {
      float val = get_point(p, rows, cols, x, y);

      if (val >= MAXTEMP)
        colorTemp = MAXTEMP;
      else if (val <= MINTEMP)
        colorTemp = MINTEMP;
      else colorTemp = val;

      uint8_t colorIndex = map(colorTemp, MINTEMP, MAXTEMP, 0, 255);
      colorIndex = constrain(colorIndex, 0, 255);// 0 ~ 255
      img_buf[y * cols * 3 + x * 3 + 0] = (camColors[colorIndex] >> 11) << 3;
      img_buf[y * cols * 3 + x * 3 + 1] = ((camColors[colorIndex] >> 5) & 0b111111) << 2;
      img_buf[y * cols * 3 + x * 3 + 2] = ((camColors[colorIndex] >> 0) & 0b11111) << 3;
    }
  }
  img_msg.header.stamp = nh.now();
  img_msg.header.frame_id = "mlx90640";
  img_msg.height = rows;
  img_msg.width = cols;
  img_msg.encoding = "rgb8";
  img_msg.is_bigendian = 0;
  img_msg.step = cols * 3;
  img_msg.data_length = rows * cols * 3;
  img_msg.data = img_buf;
  img_pub.publish(&img_msg);
}

void setup()
{
  setupM5stackROS("M5Stack ROS MLX90640");
  #ifdef DRAW_ON_LCD
    M5.Lcd.setBrightness(255);
  #else
    M5.Lcd.setBrightness(0);
  #endif
  setupMLX90640();
  nh.advertise(min_pub);
  nh.advertise(max_pub);
  nh.advertise(center_pub);
  nh.advertise(img_pub);
}

void loop()
{
  #ifdef DRAW_ON_LCD
    M5.Lcd.setBrightness(255);
  #else
    M5.Lcd.setBrightness(0);
  #endif
  measureMLX90640();
  pubMinTemp();
  pubMaxTemp();
  pubCenterTemp();
  pubImage(dest_2d, INTERPOLATED_ROWS, INTERPOLATED_COLS);
  nh.spinOnce();
}
