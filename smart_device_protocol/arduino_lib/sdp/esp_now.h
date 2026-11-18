#pragma once

#include <esp_system.h>
#include <esp_now.h>
#include <WiFi.h>

#include <smart_device_protocol/Packet.h>

bool init_esp_now(uint8_t *mac_address, esp_now_recv_cb_t callback_receive)
{
    // Read device mac address
    esp_read_mac(mac_address, ESP_MAC_WIFI_STA);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (esp_now_init() != ESP_OK)
    {
        return false;
    }
    if (callback_receive != NULL)
    {
        esp_now_register_recv_cb(callback_receive);
    }
    return true;
}