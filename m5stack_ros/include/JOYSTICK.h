// Copied from
// https://github.com/m5stack/M5Stack/blob/master/examples/Unit/JOYSTICK/JOYSTICK.ino

/*
*******************************************************************************
* Copyright (c) 2021 by M5Stack
*                  Equipped with M5Core sample source code
*                          配套  M5Core 示例源代码
* Visit the website for more
information: https://docs.m5stack.com/en/unit/joystick
* 获取更多资料请访问: https://docs.m5stack.com/zh_CN/unit/joystick
*
* describe: JOYSTICK.
* date: 2021/8/30
*******************************************************************************
  Please connect to Port A,Read JOYSTICK Unit X, Y axis offset data and button
status 请连接端口 A,读取操纵杆单位X, Y轴偏移数据和按钮状态
*/

#include <M5Stack.h>

#define JOY_ADDR 0x52  // define Joystick I2C address.  定义摇杆的I2C地址

float x_data = 0;
float y_data = 0;
float axes[2];
uint8_t button_data = 0;
int buttons[1];

void setupJoystick() {
  M5.Lcd.setCursor(70, 0, 4);
  M5.Lcd.println(("Joystick Test"));
  dacWrite(25, 0);  // disable the speak noise.  禁用语音噪音
}

void readJoystick() {
  Wire.requestFrom(
        JOY_ADDR,
        3);  // Request 3 bytes from the slave device.  向从设备请求3个字节
  if (Wire.available()) {  // If data is received.  如果接收到数据
    // axes
    // It seems that x_data_orig is 0~255 but y_data_orig is 0~238
    uint8_t x_data_orig = Wire.read();
    uint8_t y_data_orig = Wire.read();
    x_data = ((float)x_data_orig - 128) / 128;
    y_data = ((float)y_data_orig - 128) / 128;
    axes[0] = x_data;
    axes[1] = y_data;
    // buttons
    button_data = Wire.read();
    buttons[0] = button_data;
  }
}

void displayJoystick() {
  char data[100];
  sprintf(data, "x:%f y:%f button:%d\n", x_data, y_data, button_data);
  Serial.print(data);
  M5.Lcd.setCursor(100, 50, 4);
  M5.Lcd.printf("X:%f      ", x_data);
  M5.Lcd.setCursor(100, 80, 4);
  M5.Lcd.printf("Y:%f      ", y_data);
  M5.Lcd.setCursor(100, 110, 4);
  M5.Lcd.printf("B:%d      ", button_data);
}
