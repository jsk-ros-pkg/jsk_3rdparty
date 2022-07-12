// Copied from
// https://github.com/m5stack/M5Unit-ENV/blob/eaa1983195c034b98b03918e4dd3521a894f9daa/examples/Unit_ENVIII_M5Core/Unit_ENVIII_M5Core.ino
//
// Required libraries are
// - M5Unit-ENV
//   https://github.com/m5stack/M5Unit-ENV
// - Adafruit_BMP280 version 2.6.3
//   https://github.com/adafruit/Adafruit_BMP280_Library

#define ROSSERIAL_ARDUINO_BLUETOOTH

#include <m5stack_ros.h>
#include <std_msgs/Float32.h>
#include <M5_ENV_III.h>

SHT3X sht30;
QMP6988 qmp6988;

float tmp      = 0.0;
float hum      = 0.0;
float pressure = 0.0;

std_msgs::Float32 tmp_msg;
ros::Publisher tmp_pub("temperature", &tmp_msg);
std_msgs::Float32 hum_msg;
ros::Publisher hum_pub("humidity", &hum_msg);
std_msgs::Float32 pressure_msg;
ros::Publisher pressure_pub("pressure", &pressure_msg);

void setupENV() {
  M5.lcd.setTextSize(2);  // Set the text size to 2.  设置文字大小为2
  Wire.begin();  // Wire init, adding the I2C bus.  Wire初始化, 加入i2c总线
  qmp6988.init();
  M5.lcd.println(F("ENV Unit III test"));  
}

void loopENV() {
  pressure = qmp6988.calcPressure();
  if (sht30.get() == 0) {  // Obtain the data of shT30.  获取sht30的数据
    tmp = sht30.cTemp;   // Store the temperature obtained from shT30.
                         // 将sht30获取到的温度存储
    hum = sht30.humidity;  // Store the humidity obtained from the SHT30.
                           // 将sht30获取到的湿度存储
  } else {
    tmp = 0, hum = 0;
  }
  M5.lcd.fillRect(0, 20, 100, 60,
                  BLACK);  // Fill the screen with black (to clear the
                             // screen).  将屏幕填充黑色(用来清屏)
  M5.lcd.setCursor(0, 20);
  M5.Lcd.printf("Temp: %2.1f  \r\nHumi: %2.0f%%  \r\nPressure:%2.0fPa\r\n",
                tmp, hum, pressure);  
}

void publishENV() {
  tmp_msg.data = tmp;
  hum_msg.data = hum;
  pressure_msg.data = pressure;
  tmp_pub.publish(&tmp_msg);
  hum_pub.publish(&hum_msg);
  pressure_pub.publish(&pressure_msg);
}

void setup() {
  setupM5stackROS("M5Stack ROS ENVIII");
  setupENV();

  nh.advertise(tmp_pub);
  nh.advertise(hum_pub);
  nh.advertise(pressure_pub);
}

void loop() {
  loopENV();
  publishENV();
  nh.spinOnce();
  delay(10);
}
