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
/*
*******************************************************************************
* Copyright (c) 2022 by M5Stack
*                  Equipped with M5Core sample source code
*                          配套  M5Core 示例源代码
* Visit for more information: https://docs.m5stack.com/en/core/gray
* 获取更多资料请访问: https://docs.m5stack.com/zh_CN/core/gray
*
* Describe: Button_Two.  双按键
* Date: 2021/8/9
******************************************** ***********************************
  Please connect to Port B(36、26),Read the button status of BUTTON Unit and
display it on the screen 请连接端口B(36、26),读取按键的状态并在显示屏上显示 if
you don't have M5GO BOTTOM, you need change the pinMode and the digitalRead to
22、21, But you will not be able to use any I2C operations. 如果你没有M5GO
BOTTOM，你需要改变pinMode和digitalRead到22,21,但是你将不能使用任何I2C操作.
*/
// Copied from
// https://github.com/m5stack/M5Stack/blob/master/examples/Unit/DUAL_BUTTON/DUAL_BUTTON.ino

#include <M5Stack.h>

int pin1 = 36, pin2 = 26;
int last_value1 = 0, last_value2 = 0;
int cur_value1 = 0, cur_value2 = 0;
int buttons[2];

void setupDualButton(int pin1_, int pin2_) {
  pin1 = pin1_;
  pin2 = pin2_;
  M5.begin();        // Init M5Stack.  初始化M5Stack
  M5.Power.begin();  // Init power  初始化电源模块
  pinMode(pin1, INPUT);  // set pin mode to input.设置引脚模式为输入模式
  pinMode(pin2, INPUT);
  M5.Lcd.setTextColor(
                      YELLOW);  // Set the font color to yellow.  设置字体颜色为黄色
  M5.Lcd.setTextSize(2);  // Setting the Font size.  设置字号大小
  M5.Lcd.setCursor(
                   80, 0);  // Set the cursor position to (80,0).  将光标位置设置为(80,0)
  M5.Lcd.println("Button example");
  M5.Lcd.setTextColor(WHITE);
}

void readDualButton() {
  cur_value1 = digitalRead(pin1);  // read the value of BUTTON. 读取22号引脚的值
  cur_value2 = digitalRead(pin2);
  buttons[0] = cur_value1;
  buttons[1] = cur_value2;
}

void displayDualButton() {
  M5.Lcd.setCursor(90, 25);
  M5.Lcd.print("Btn.1  Btn.2");
  M5.Lcd.setCursor(0, 45);
  M5.Lcd.print("Value: ");
  M5.Lcd.setCursor(0, 85);
  M5.Lcd.print("State: ");
  if (cur_value1 != last_value1) {
    M5.Lcd.fillRect(85, 45, 75, 85,
                    BLACK);  // Draw a black rectangle 75 by 85 at (85,45).
    // 在(85,45)处绘制宽75,高85的黑色矩形
    if (cur_value1 == 0) {
      M5.Lcd.setCursor(95, 45);
      M5.Lcd.print("0");  // display the status
      M5.Lcd.setCursor(95, 85);
      M5.Lcd.print("pre");
    } else {
      M5.Lcd.setCursor(95, 45);
      M5.Lcd.print("1");  // display the status
      M5.Lcd.setCursor(95, 85);
      M5.Lcd.print("rel");
    }
    last_value1 = cur_value1;
  }
  if (cur_value2 != last_value2) {
    M5.Lcd.fillRect(170, 45, 75, 85, BLACK);
    if (cur_value2 == 0) {
      M5.Lcd.setCursor(185, 45);
      M5.Lcd.print("0");  // display the status
      M5.Lcd.setCursor(185, 85);
      M5.Lcd.print("pre");
    } else {
      M5.Lcd.setCursor(185, 45);
      M5.Lcd.print("1");  // display the status
      M5.Lcd.setCursor(185, 85);
      M5.Lcd.print("rel");
    }
    last_value2 = cur_value2;
  }
}
