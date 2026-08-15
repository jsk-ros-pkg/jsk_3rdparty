#ifndef STICKV2_UTIL_H
#define STICKV2_UTIL_H

#include <Arduino.h>

#include <ArduinoJson.h>

#define BUFSIZE 2048

bool send_data_to_serial(HardwareSerial &serial, StaticJsonDocument<BUFSIZE> &doc)
{
    String request;
    serializeJson(doc, request);
    serial.println(request);
    return true;
}

bool read_data_from_serial(HardwareSerial &serial, StaticJsonDocument<BUFSIZE> &doc)
{
    String response = serial.readStringUntil('\n');
    DeserializationError error = deserializeJson(doc, response);
    if (error)
    {
        return false;
    }
    return true;
}

bool set_object_recognition_model(HardwareSerial &serial, const String &model_path)
{
    StaticJsonDocument<BUFSIZE> doc;
    doc["function"] = "object_recognition";
    doc["args"][0] = model_path;
    return send_data_to_serial(serial, doc);
}

#endif