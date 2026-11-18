// The MIT License (MIT)

// Copyright (c) 2013 Seeed Technology Inc.

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

/*
    Grove LED Bar - Basic Control Example
    This example will show you how to use the setBits() function of this library.
    Set any combination of LEDs using 10 bits.
    Least significant bit controls the first LED.
    The setBits() function sets the current state, one bit for each LED.
    First 10 bits from the right control the 10 LEDs.
    eg. 0b00000jihgfedcba
    a = LED 1, b = LED 2, c = LED 3, etc.
    dec    hex     binary
    0    = 0x0   = 0b000000000000000 = all LEDs off
    5    = 0x05  = 0b000000000000101 = LEDs 1 and 3 on, all others off
    341  = 0x155 = 0b000000101010101 = LEDs 1,3,5,7,9 on, 2,4,6,8,10 off
    1023 = 0x3ff = 0b000001111111111 = all LEDs on
                      |        |
                      10       1
    The bits >10 are ignored, shown here as x: 0bxxxxx0000000000
*/

// Mainly copied from
// https://github.com/Seeed-Studio/Grove_LED_Bar/blob/b2964c4f9d967a0c891d25432cbc7ce83f3832ed/examples/BasicControl/BasicControl.ino

#include <Grove_LED_Bar.h>
// Args of bar(): Clock pin, Data pin, Orientation
// For TimerCam Grove connector
Grove_LED_Bar bar(13, 4, 0, LED_CIRCULAR_24);
// For M5Stack Gray Grove connector
// Grove_LED_Bar bar(22, 21, 0, LED_BAR_10);

void setupGroveCircularLED() {
  // nothing to initialize
  bar.begin();
}

void turnOnGroveCircularLED () {
  // Turn on all LEDs
  // 24 bits corresponds to on/off of 24 LED
  bar.setBits(0xffffff);
}

void turnOffGroveCircularLED () {
  // Try until all LEDs are confirmed to be off
  while ( 0 != bar.getBits()) {
    // Turn off all LEDs
    // 24 bits corresponds to on/off of 24 LED
    bar.setBits(0x000000);
    delay(100);
  }
}
