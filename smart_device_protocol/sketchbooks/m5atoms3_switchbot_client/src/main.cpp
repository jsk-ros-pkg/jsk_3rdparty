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
#include "web_services/switchbot_util.h"

/* Mofidy below */
String ssid = "";
String password = "";
String token = "";
String secret = "";

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
  sprite_device_info.println("Switchbot Client");
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
    if (command == String("get_device_list"))
    {
      std::optional<String> ret = get_device_list(token, secret);
      if (ret)
      {
        DeserializationError error = deserializeJson(result_json, ret.value().c_str());
        if (error)
        {
          response_json["success"] = false;
          response_json["message"] = "deserializeJson() failed during get_device_list: " + String(error.c_str()) + ", ret: " + ret.value();
        }
        else
        {
          response_json["success"] = true;
          response_json["message"] = "get_device_list success";
          response_json["result"] = result_json;
        }
      }
      else
      {
        response_json["success"] = false;
        response_json["message"] = "get_device_list failed";
      }
    }
    else if (command == String("get_device_status"))
    {
      if (not input_json.containsKey("device_id"))
      {
        response_json["success"] = false;
        response_json["message"] = "device_id key not found for get_device_status";
      }
      else
      {
        String device_id = input_json["device_id"];
        std::optional<String> ret = get_device_status(token, secret, device_id);
        if (ret)
        {
          DeserializationError error = deserializeJson(result_json, ret.value().c_str());
          if (error)
          {
            response_json["success"] = false;
            response_json["message"] = "deserializeJson() failed during get_device_status: " + String(error.c_str()) + ", ret: " + ret.value();
          }
          else
          {
            response_json["success"] = true;
            response_json["message"] = "get_device_status success";
            response_json["result"] = result_json;
          }
        }
        else
        {
          response_json["success"] = false;
          response_json["message"] = "get_device_status failed";
        }
      }
    }
    else if (command == "send_device_command")
    {
      if (not input_json.containsKey("device_id") or not input_json.containsKey("sb_command_type") or not input_json.containsKey("sb_command"))
      {
        response_json["success"] = false;
        response_json["message"] = "device_id or sb_command_type or sb_command key not found for send_device_command";
      }
      else
      {
        String device_id = input_json["device_id"];
        String sb_command_type = input_json["sb_command_type"];
        String sb_command = input_json["sb_command"];
        std::optional<String> ret = send_device_command(token, secret, device_id, sb_command_type, sb_command);
        if (ret)
        {
          DeserializationError error = deserializeJson(result_json, ret.value().c_str());
          if (error)
          {
            response_json["success"] = false;
            response_json["message"] = "deserializeJson() failed during send_device_command: " + String(error.c_str()) + ", ret: " + ret.value();
          }
          else
          {
            response_json["success"] = true;
            response_json["message"] = "send_device_command success";
            response_json["result"] = result_json;
          }
        }
        else
        {
          response_json["success"] = false;
          response_json["message"] = "send_device_command failed";
        }
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
    else if (command == String("config_switchbot"))
    {
      if (not input_json.containsKey("token") or not input_json.containsKey("secret"))
      {
        response_json["success"] = false;
        response_json["message"] = "token or secret key not found";
      }
      else
      {
        token = input_json["token"].as<String>();
        secret = input_json["secret"].as<String>();
        response_json["success"] = true;
        response_json["message"] = "config_switchbot success";
      }
    }
    else if (command == String("get_device_config"))
    {
      result_json["ssid"] = ssid;
      result_json["password"] = password;
      result_json["token"] = token;
      result_json["secret"] = secret;
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
