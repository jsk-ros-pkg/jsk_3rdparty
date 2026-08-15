#ifndef UWB_MODULE_UTIL_H
#define UWB_MODULE_UTIL_H

#include <Arduino.h>
#include <optional>

/**
 * @brief Drivers for UWB module
 * http://docs.m5stack.com/en/unit/uwb
 */

std::optional<String> readUWB(HardwareSerial& serial, int timeout);
String testUWB(HardwareSerial& serial);
bool resetUWB(HardwareSerial& serial);
bool initUWB(bool tag, int id, HardwareSerial& serial);

std::optional<String> readUWB(HardwareSerial& serial, int timeout = 1000)
{
  String DATA;
  auto start = millis();
  while (timeout > millis() - start)
  {
    if (serial.available())
    {
      DATA = serial.readStringUntil('\n');
      return DATA;
    }
  }
  return std::nullopt;
}

std::optional<std::tuple<int, float>> getDistanceUWB(HardwareSerial& serial)
{
  std::optional<String> ret = readUWB(serial);
  if (ret)
  {
    auto data = *ret;
    auto index = data.indexOf(':');
    if (index != -1)
    {
      auto id_str = data.substring(0, index);
      id_str.replace(String("an"), String(""));
      auto id = id_str.toInt();
      auto distance_str = data.substring(index + 1);
      distance_str.replace(String("m"), String(""));
      auto distance = distance_str.toFloat();
      return std::make_tuple(id, distance);
    }
  }
  return std::nullopt;
}

void clearReadUWB(HardwareSerial& serial, int timeout = 1000)
{
  auto start = millis();
  while (timeout > millis() - start)
  {
    if (serial.available())
    {
      serial.readStringUntil('\n');
    }
  }
}

String testUWB(HardwareSerial& serial)
{
  serial.write("AT\r\n");
  delay(100);
  auto ret = readUWB(serial);
  if (ret)
  {
    return *ret;
  }
  else
  {
    return "No response";
  }
}

bool resetUWB(HardwareSerial& serial)
{
  serial.write("AT+RST\r\n");
  delay(500);
  auto ret = readUWB(serial);
  clearReadUWB(serial);
  if (ret)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool initUWB(bool tag, int id, HardwareSerial& serial)
{
  String DATA;
  auto ret = resetUWB(serial);
  if (!ret)
  {
    return false;
  }
  if (tag)
  {
    serial.printf("AT+anchor_tag=0\r\n", id);
    delay(100);
    readUWB(serial);

    resetUWB(serial);

    serial.write("AT+interval=1\r\n");
    delay(100);
    readUWB(serial);

    serial.write("AT+switchdis=1\r\n");
    delay(100);
    readUWB(serial);
  }
  else
  {
    serial.printf("AT+anchor_tag=1,%d\r\n", id);
    delay(100);
    readUWB(serial);

    resetUWB(serial);
  }

  return true;
}

#endif  // UWB_MODULE_UTIL_H
