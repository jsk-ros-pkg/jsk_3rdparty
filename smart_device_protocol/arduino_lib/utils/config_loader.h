#ifndef SMART_DEVICE_PROTOCOL_CONFIG_LOADER_H
#define SMART_DEVICE_PROTOCOL_CONFIG_LOADER_H

#include <Arduino.h>
#include <ArduinoJson.h>

template <int N>
bool load_json_from_FS(fs::FS &fs, const String &filename, StaticJsonDocument<N> &doc)
{
    auto file = fs.open(filename.c_str());
    if (!file)
    {
        Serial.printf("Failed to open config file from %s\n", filename.c_str());
        file.close();
        return false;
    }
    DeserializationError error = deserializeJson(doc, file.readString());
    if (error)
    {
        Serial.println("Failed to parse config file");
        file.close();
        return false;
    }
    file.close();
    return true;
}

#endif // SMART_DEVICE_PROTOCOL_CONFIG_LOADER_H