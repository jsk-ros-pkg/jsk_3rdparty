int gas_ain=36;
int analog_value;
int digital_value;
void setup()
{
  pinMode(gas_din,INPUT);
  pinMode(gas_ain,INPUT);
  Serial.begin(115200);
}
void loop()
{
  analog_value=analogRead(gas_ain);

  Serial.print("analog_value:");
  Serial.println(analog_value);

  delay(500);
}
