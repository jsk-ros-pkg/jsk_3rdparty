/*
MIT License

Copyright (c) 2021 M5Stack

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
// https://github.com/m5stack/M5Unit-ENV/blob/0.0.5/examples/Unit_ENVIII_M5Core/Unit_ENVIII_M5Core.ino

#include <M5Stack.h>
#include <M5_ENV.h>
#include <print.h>

SHT3X sht30;
QMP6988 qmp6988;

float tmp      = 0.0;
float hum      = 0.0;
float pressure = 0.0;

void setupENV() {
  M5.lcd.setTextSize(2);  // Set the text size to 2.  设置文字大小为2
  Wire.begin();  // Wire init, adding the I2C bus.  Wire初始化, 加入i2c总线
  qmp6988.init();
  M5.lcd.println(F("ENV Unit III test"));
}

void measureENV() {
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
  char buffer[100];
  sprintf(buffer, "Temp: %2.1f  \r\nHumi: %2.0f%%  \r\nPressure:%2.0fPa\r\n",
          tmp, hum, pressure);
  M5.Lcd.printf(buffer);
  PRINTLN(buffer);
}
