int gas_din=26;
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
  digital_value=digitalRead(gas_din);

  Serial.print("analog_value:");
  Serial.print(analog_value);
  Serial.print(", digital_value:");
  Serial.println(digital_value);

  delay(500);
}
