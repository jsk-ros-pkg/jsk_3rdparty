#ifndef IOT_HOST_UTIL_H
#define IOT_HOST_UTIL_H

#include <Arduino.h>

String send_serial_command(String command, int timeout_duration = 5000)
{
  Serial2.print(command.c_str());
  auto timeout = millis() + timeout_duration;
  while (millis() < timeout)
  {
    delay(100);
    if (Serial2.available())
    {
      return Serial2.readStringUntil('\n');
    }
  }
  return "";
}

void clear_recv_buf(int duration = 5000)
{
  auto timeout = millis() + duration;
  while (millis() < timeout)
  {
    delay(100);
    if (Serial2.available())
    {
      Serial2.readStringUntil('\n');
    }
  }
  return;
}

#endif // IOT_HOST_UTIL_H
