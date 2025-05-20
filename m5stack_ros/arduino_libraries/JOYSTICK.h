/*
  MIT License

  Copyright (c) 2017 M5Stack

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

// Copied from
// https://github.com/m5stack/M5Stack/blob/master/examples/Unit/JOYSTICK/JOYSTICK.ino

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
