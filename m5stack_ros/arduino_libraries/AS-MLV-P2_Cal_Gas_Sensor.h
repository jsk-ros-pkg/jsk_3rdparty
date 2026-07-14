#include <M5Stack.h>
#include <Wire.h>

int gas_ain=36;
uint16_t analog_value;

void setupCalGas()
{
  Wire.begin();

  pinMode(gas_ain,INPUT);
}

void measureCalGas()
{
  analog_value=analogRead(gas_ain);
}

void displayCalGas()
{
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(10, 10);

  M5.Lcd.printf("analog_value: %04d\n", analog_value);
}
