#include <optional>

#include <M5AtomS3.h>

#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>

#include <ArduinoJson.h>

#define LGFX_AUTODETECT
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>

#include "iot_com_util/Time.h"
#include "iot_com_util/iot_client_util.h"
#include "web_services/sesami_util.h"

/* Mofidy below */
String ssid = "";
String password = "";
String device_uuid = "";
String secret_key = "";
String api_key = "";

//
WiFiMulti WiFiMulti;
static LGFX lcd;
static LGFX_Sprite sprite_device_info(&lcd);
static LGFX_Sprite sprite_event_info(&lcd);

void setup()
{
  M5.begin(false, true, false, false);
  Serial2.begin(115200, SERIAL_8N1, 2, 1);

  // LovyanGFX Initialization
  init_screen(lcd, sprite_device_info, sprite_event_info);

  sprite_device_info.setCursor(0, 0);
  sprite_device_info.println("SESAMI Client");
  sprite_device_info.pushSprite(0, 0);

  if (ssid != "" and password != "")
  {
    initWiFi(ssid.c_str(), password.c_str(), sprite_event_info, lcd, WiFiMulti);
  }
}

void loop()
{
  delay(50);
  String bufstring;
  StaticJsonDocument<1024> input_json;
  StaticJsonDocument<1024> result_json;
  StaticJsonDocument<2048> response_json;
  if (USBSerial.available() or Serial2.available())
  {
    if (USBSerial.available())
    {
      bufstring = USBSerial.readStringUntil('\n');
    }
    else
    {
      bufstring = Serial2.readStringUntil('\n');
    }
    DeserializationError error = deserializeJson(input_json, bufstring.c_str());

    if (error)
    {
      String message = "deserializeJson() failed: " + String(error.c_str());
      USBSerial.println(message);
      show_device_info(message.c_str(), sprite_event_info, lcd);
      response_json["success"] = false;
      response_json["message"] = message;
      Serial2.printf("%s\n", response_json.as<String>().c_str());
      return;
    }

    if (not input_json.containsKey("command"))
    {
      String message = "command key not found";
      USBSerial.println(message);
      show_device_info(message.c_str(), sprite_event_info, lcd);
      response_json["success"] = false;
      response_json["message"] = message;
      Serial2.printf("%s\n", response_json.as<String>().c_str());
      return;
    }

    String command = input_json["command"];
    if (command == String("toggle") or command == String("lock") or command == String("unlock"))
    {
      std::optional<String> ret;
      if (command == String("toggle"))
      {
        ret = operation_sesami(device_uuid, api_key, 88, secret_key);
      }
      else if (command == String("lock"))
      {
        ret = operation_sesami(device_uuid, api_key, 82, secret_key);
      }
      else if (command == String("unlock"))
      {
        ret = operation_sesami(device_uuid, api_key, 83, secret_key);
      }
      if (ret)
      {
        DeserializationError error = deserializeJson(result_json, ret.value().c_str());
        if (error)
        {
          response_json["success"] = false;
          response_json["message"] = "deserializeJson() failed during operation_sesami: " + String(error.c_str()) + ", result: " + ret.value();
        }
        else
        {
          response_json["success"] = true;
          response_json["message"] = command + " success";
          response_json["result"] = result_json;
        }
      }
      else
      {
        response_json["success"] = false;
        response_json["message"] = command + " failed";
      }
    }
    else if (command == String("status"))
    {
      std::optional<String> ret = get_sesami_status(device_uuid, api_key);
      if (ret)
      {
        DeserializationError error = deserializeJson(result_json, ret.value().c_str());
        if (error)
        {
          response_json["success"] = false;
          response_json["message"] = "deserializeJson() failed during get_sesami_status: " + String(error.c_str()) + ", result: " + ret.value();
        }
        else
        {
          response_json["success"] = true;
          response_json["message"] = "get_sesami_status() success";
          response_json["result"] = result_json;
        }
      }
      else
      {
        response_json["success"] = false;
        response_json["message"] = "get_sesami_status() failed";
      }
    }
    else if (command == String("history"))
    {
      std::optional<String> ret = get_sesami_history(device_uuid, api_key);
      if (ret)
      {
        String result = ret.value();
        DeserializationError error = deserializeJson(result_json, result.c_str());
        if (error)
        {
          response_json["success"] = false;
          response_json["message"] = "deserializeJson() failed during get_sesami_history: " + String(error.c_str()) + ", result: " + result;
        }
        else
        {
          response_json["success"] = true;
          response_json["message"] = "get sesami history success";
          response_json["result"] = result_json;
        }
      }
      else
      {
        response_json["success"] = false;
        response_json["message"] = "get_sesami_history() failed";
      }
    }
    else if (command == "get_time")
    {
      uint32_t t = (uint32_t)std::time(nullptr);
      response_json["success"] = true;
      response_json["message"] = "get_time success: " + String(t);
    }
    else if (command == String("config_wifi"))
    {
      if (not input_json.containsKey("ssid") or not input_json.containsKey("password"))
      {
        response_json["success"] = false;
        response_json["message"] = "ssid or password key not found";
      }
      else
      {
        String new_ssid = input_json["ssid"].as<String>();
        String new_password = input_json["password"].as<String>();
        if (new_ssid != "")
          ssid = new_ssid;
        if (new_password != "")
          password = new_password;
        bool success = initWiFi(ssid.c_str(), password.c_str(), sprite_event_info, lcd, WiFiMulti);
        String message;
        if (success)
        {
          message = "config_wifi success. SSID: " + ssid + ", password: " + password + ", IP: " + WiFi.localIP().toString();
        }
        else
        {
          message = "config_wifi failed";
        }
        response_json["success"] = success;
        response_json["message"] = message;
      }
    }
    else if (command == String("config_sesami"))
    {
      if (not input_json.containsKey("device_uuid") or not input_json.containsKey("secret_key") or not input_json.containsKey("api_key"))
      {
        response_json["success"] = false;
        response_json["message"] = "device_uuid or secret_key or api_key key not found";
      }
      else
      {
        device_uuid = input_json["device_uuid"].as<String>();
        secret_key = input_json["secret_key"].as<String>();
        api_key = input_json["api_key"].as<String>();
        response_json["success"] = true;
        response_json["message"] = "config_sesami success";
      }
    }
    else if (command == String("get_device_config"))
    {
      result_json["ssid"] = ssid;
      result_json["password"] = password;
      result_json["device_uuid"] = device_uuid;
      result_json["secret_key"] = secret_key;
      result_json["api_key"] = api_key;
      result_json["ip"] = WiFi.localIP().toString();
      response_json["success"] = true;
      response_json["message"] = "get_device_config success";
      response_json["result"] = result_json;
    }
    else
    {
      response_json["success"] = false;
      response_json["message"] = "Unknown command error: " + command;
    }
    USBSerial.printf("response_json: %s\n", response_json.as<String>().c_str());
    show_device_info((String("response_json") + response_json.as<String>()).c_str(), sprite_event_info, lcd);
    Serial2.printf("%s\n", response_json.as<String>().c_str());
  }
}
