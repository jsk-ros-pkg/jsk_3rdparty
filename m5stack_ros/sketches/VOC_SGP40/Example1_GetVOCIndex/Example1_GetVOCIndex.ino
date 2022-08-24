/*
  Library for the Sensirion SGP40 Indoor Air Quality Sensor
  By: Paul Clark
  SparkFun Electronics
  Date: December 5th, 2020
  License: please see LICENSE.md for details

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/17729
*/

#include "SparkFun_SGP40_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_SGP40
#include <Wire.h>

SGP40 mySensor; //create an object of the SGP40 class

void setup()
{
  Serial.begin(115200);
  Serial.println(F("SGP40 Example"));

  Wire.begin();

  mySensor.enableDebugging(); // Uncomment this line to print useful debug messages to Serial

  //Initialize sensor
  if (mySensor.begin() == false)
  {
    Serial.println(F("SGP40 not detected. Check connections. Freezing..."));
    while (1)
      ; // Do nothing more
  }
}

void loop()
{
  Serial.print(F("VOC Index is: "));
  Serial.println(mySensor.getVOCindex()); //Get the VOC Index using the default RH (50%) and T (25C)

  delay(1000); //Wait 1 second - the Sensirion VOC algorithm expects a sample rate of 1Hz
}
