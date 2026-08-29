/*
    Multichannel_gas_sensor_V2.0.ino
    Description: A terminal for Seeed Grove Multichannel gas sensor V2.0.
    2019 Copyright (c) Seeed Technology Inc.  All right reserved.
    Author: Hongtai Liu(lht856@foxmail.com)
    2019-9-29

    The MIT License (MIT)
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:
    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.1  USA
*/

#include <M5Stack.h>
#include <Multichannel_Gas_GMXXX.h>

// if you use the software I2C to drive the sensor, you can uncommnet the define SOFTWAREWIRE which in Multichannel_Gas_GMXXX.h.
#ifdef SOFTWAREWIRE
    #include <SoftwareWire.h>
    SoftwareWire myWire(3, 2);
    GAS_GMXXX<SoftwareWire> gas;
#else
    #include <Wire.h>
    GAS_GMXXX<TwoWire> gas;
#endif

static uint8_t recv_cmd[8] = {};
uint32_t val_102B = 0;
uint32_t val_302B = 0;
uint32_t val_502B = 0;
uint32_t val_702B = 0;

void setupGasV2()
{
  // If you have changed the I2C address of gas sensor, you must to be specify the address of I2C.
  //The default addrss is 0x08;
  gas.begin(Wire, 0x08); // use the hardware I2C
  //gas.begin(MyWire, 0x08); // use the software I2C
  //gas.setAddress(0x64); change thee I2C address
}

void measureGasV2()
{
  val_102B=gas.getGM102B();
  val_302B=gas.getGM302B();
  val_502B=gas.getGM502B();
  val_702B=gas.getGM702B();
}

void displayGasV2()
{
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(10, 10);

  M5.Lcd.printf("GM102B: %4u = %.2f V\n", val_102B, gas.calcVol(val_102B));
  M5.Lcd.printf(" GM302B: %4u = %.2f V\n", val_302B, gas.calcVol(val_302B));
  M5.Lcd.printf(" GM502B: %4u = %.2f V\n", val_502B, gas.calcVol(val_502B));
  M5.Lcd.printf(" GM702B: %4u = %.2f V\n", val_702B, gas.calcVol(val_702B));
}
