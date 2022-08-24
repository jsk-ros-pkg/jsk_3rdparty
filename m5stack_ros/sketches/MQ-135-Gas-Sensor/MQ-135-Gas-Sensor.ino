int gas_din=26;
int gas_ain=36;
int ad_value;
void setup()
{
  pinMode(gas_din,INPUT);
  pinMode(gas_ain,INPUT);
  Serial.begin(115200);
}
void loop()
{
  ad_value=analogRead(gas_ain);
  if(digitalRead(gas_din)==LOW)
  {
    Serial.println("Gas leakage");
    Serial.print("ad_value:");
    Serial.print(ad_value*3.3/1024);
    Serial.println("V");
  }
  else
  {
    Serial.println("Gas not leak");
  }
  delay(500);
}
