// Copied from https://github.com/start-jsk/jsk_apc/blob/16c004c511e864478aa6581a04c5a023e5fde391/demos/sphand_ros/sphand_driver/arduino/test_palm_v8/test_palm_v8.ino
// Also check https://github.com/RoboticMaterials/FA-I-sensor/blob/master/force_proximity_ros/src/proximity_sensor_vcnl4040/proximity_sensor_vcnl4040.ino

#include <M5Stack.h>
#include <Wire.h>

/***** GLOBAL CONSTANTS *****/
#define VCNL4040_ADDR 0x60 //7-bit unshifted I2C address of VCNL4040
//Command Registers have an upper byte and lower byte.
#define PS_CONF1 0x03
//#define PS_CONF2 //High byte of PS_CONF1
#define PS_CONF3 0x04
//#define PS_MS //High byte of PS_CONF3
#define PS_DATA_L 0x08
//#define PS_DATA_M //High byte of PS_DATA_L

uint16_t proximity;

//Reads a two byte value from a command register
unsigned int readFromCommandRegister(byte commandCode)
{
  Wire.beginTransmission(VCNL4040_ADDR);
  Wire.write(commandCode);
  Wire.endTransmission(false); //Send a restart command. Do not release bus.

  Wire.requestFrom(VCNL4040_ADDR, 2); //Command codes have two bytes stored in them

  unsigned int data = Wire.read();
  data |= Wire.read() << 8;

  return (data);
}

//Write a two byte value to a Command Register
void writeToCommandRegister(byte commandCode, byte lowVal, byte highVal)
{
  Wire.beginTransmission(VCNL4040_ADDR);
  Wire.write(commandCode);
  Wire.write(lowVal); //Low byte of command
  Wire.write(highVal); //High byte of command
  Wire.endTransmission(); //Release bus
}

void setupVCNL4040()
{
  Wire.begin();
  delay(1);
  //Set the options for PS_CONF3 and PS_MS bytes
  byte conf3 = 0x00;
  byte ms = 0b00000001; //Set IR LED current to 75mA
  //byte ms = 0b00000010; //Set IR LED current to 100mA
  //byte ms = 0b00000110; //Set IR LED current to 180mA
  //byte ms = 0b00000111; //Set IR LED current to 200mA
  writeToCommandRegister(PS_CONF3, conf3, ms);
}

void startProxSensor()
{
  //Clear PS_SD to turn on proximity sensing
  //byte conf1 = 0b00000000; //Clear PS_SD bit to begin reading
  byte conf1 = 0b00001110; //Integrate 8T, Clear PS_SD bit to begin reading
  byte conf2 = 0b00001000; //Set PS to 16-bit
  //byte conf2 = 0b00000000; //Clear PS to 12-bit
  writeToCommandRegister(PS_CONF1, conf1, conf2); //Command register, low byte, high byte
}

void stopProxSensor()
{
  //Set PS_SD to turn off proximity sensing
  byte conf1 = 0b00000001; //Set PS_SD bit to stop reading
  byte conf2 = 0b00000000;
  writeToCommandRegister(PS_CONF1, conf1, conf2); //Command register, low byte, high byte
}

void measureVCNL4040() {
  startProxSensor();
  delay(10);
  proximity = readFromCommandRegister(PS_DATA_L);
  stopProxSensor();
}

void displayVCNL4040()
{
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(10, 10);
  M5.Lcd.printf("proximity: %5u", proximity);
}
