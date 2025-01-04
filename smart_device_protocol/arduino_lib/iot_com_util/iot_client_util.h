#ifndef IOT_WIFI_UTIL_H
#define IOT_WIFI_UTIL_H

#include <WiFi.h>
#include <WiFiMulti.h>

#include <ESP32Ping.h>
#include <LovyanGFX.hpp>

#include "iot_com_util/Time.h"

bool initWiFi(const char *ssid, const char *password, LGFX_Sprite &sprite, LGFX &lcd, WiFiMulti &WiFiMulti)
{
    WiFiMulti.addAP(ssid, password);
    sprite.fillScreen(0xFFFFFF);
    sprite.setCursor(0, 0);
    sprite.printf("Waiting connect to WiFi: %s ...", ssid);
    sprite.pushSprite(0, lcd.height() / 3);
    int sum = 0;
    while (WiFiMulti.run() != WL_CONNECTED)
    {
        sprite.print(".");
        sprite.pushSprite(0, lcd.height() / 3);
        delay(1000);
        sum += 1;
        if (sum == 5)
        {
            sprite.print("Conncet failed!");
            sprite.pushSprite(0, lcd.height() / 3);
            return false;
        }
    }

    if (not Ping.ping("www.google.com"))
    {
        sprite.fillScreen(0xFFFFFF);
        sprite.setCursor(0, 0);
        sprite.print("Ping failed!");
        sprite.pushSprite(0, lcd.height() / 3);
        return false;
    }
    sprite.fillScreen(0xFFFFFF);
    sprite.println("WiFi connected");
    sprite.print("IP address: ");
    sprite.println(WiFi.localIP());
    sprite.pushSprite(0, lcd.height() / 3);
    Time.set_time();
    delay(500);
    return true;
}

void init_screen(LGFX &lcd, LGFX_Sprite &sprite_device_info, LGFX_Sprite &sprite_event_info)
{
    lcd.init();
    lcd.setRotation(1);
    lcd.setBrightness(128);
    lcd.setColorDepth(24);
    lcd.fillScreen(0xFFFFFF);
    sprite_device_info.createSprite(lcd.width(), lcd.height() / 3);
    sprite_event_info.createSprite(lcd.width(), lcd.height() / 3 * 2);
    sprite_device_info.fillScreen(0xFFFFFF);
    sprite_event_info.fillScreen(0xFFFFFF);
    sprite_device_info.setTextColor(0x000000);
    sprite_event_info.setTextColor(0x000000);
    sprite_device_info.setTextSize(1);
    sprite_event_info.setTextSize(1);
}

void show_device_info(const char *message, LGFX_Sprite &sprite, LGFX &lcd)
{
    sprite.fillScreen(0xFFFFFF);
    sprite.setCursor(0, 0);
    sprite.println(message);
    sprite.pushSprite(0, lcd.height() / 3);
}

#endif // IOT_WIFI_UTIL_H