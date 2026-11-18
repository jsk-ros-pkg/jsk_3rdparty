#ifndef SESAMI_UTIL_H
#define SESAMI_UTIL_H

#include <M5AtomS3.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>

#include <AES_CMAC.h>
#include <AES.h>

#include <base64.h>

#include <optional>

String generateRandomTag(String secret_key, uint32_t date_sec);

std::optional<String> operation_sesami(String device_uuid, String api_key, int command, String secret_key);

std::optional<String> get_sesami_status(String device_uuid, String api_key);

std::optional<String> get_sesami_history(String device_uuid, String api_key);

String generateRandomTag(String secret_key, uint32_t date_sec)
{
    AESTiny128 aes128;
    AES_CMAC cmac(aes128);

    uint8_t dateBytes[3];
    dateBytes[0] = (date_sec >> 8) & 0xFF;
    dateBytes[1] = (date_sec >> 16) & 0xFF;
    dateBytes[2] = (date_sec >> 24) & 0xFF;

    uint8_t key[16];
    for (int i = 0; i < 16; i++)
    {
        key[i] = 0;
        if (secret_key[2 * i] >= '0' and secret_key[2 * i] <= '9')
        {
            key[i] += (secret_key[2 * i] - '0') << 4;
        }
        else if (secret_key[2 * i] >= 'a' and secret_key[2 * i] <= 'f')
        {
            key[i] += (secret_key[2 * i] - 'a' + 10) << 4;
        }
        else if (secret_key[2 * i] >= 'A' and secret_key[2 * i] <= 'F')
        {
            key[i] += (secret_key[2 * i] - 'A' + 10) << 4;
        }
        else
        {
            USBSerial.printf("secret_key[%d]: %c\n", 2 * i, secret_key[2 * i]);
            USBSerial.println("secret_key error!");
        }

        if (secret_key[2 * i + 1] >= '0' and secret_key[2 * i + 1] <= '9')
        {
            key[i] += (secret_key[2 * i + 1] - '0');
        }
        else if (secret_key[2 * i + 1] >= 'a' and secret_key[2 * i + 1] <= 'f')
        {
            key[i] += (secret_key[2 * i + 1] - 'a' + 10);
        }
        else if (secret_key[2 * i + 1] >= 'A' and secret_key[2 * i + 1] <= 'F')
        {
            key[i] += (secret_key[2 * i + 1] - 'A' + 10);
        }
        else
        {
            USBSerial.printf("secret_key[%d]: %c\n", 2 * i + 1, secret_key[2 * i + 1]);
            USBSerial.println("secret_key error!");
        }
    }

    uint8_t output[16];

    cmac.generateMAC(output, key, dateBytes, sizeof(dateBytes));

    char outputHex[32];
    for (int i = 0; i < 16; i++)
    {
        sprintf(outputHex + (i * 2), "%02x", output[i]);
    }
    return String(outputHex);
}

std::optional<String> operation_sesami(String device_uuid, String api_key, int command, String secret_key)
{
    uint32_t date_sec = time(nullptr);
    String sign = generateRandomTag(secret_key, date_sec);
    String history = "test02";
    String base64History = base64::encode(history);
    String body = String("{\"cmd\":") + command + String(",\"sign\":\"") + sign + String("\",\"history\":\"") +
                  base64History + String("\"}");
    String url = String("https://app.candyhouse.co/api/sesame2/") + device_uuid + String("/cmd");

    HTTPClient http;

    if (!http.begin(url))
    {
        return std::nullopt;
    }

    USBSerial.println("=== HTTPClient begin! ===");
    USBSerial.printf("URL: %s\n", url.c_str());
    USBSerial.printf("date: %d\n", date_sec);
    USBSerial.printf("sign: %s\n", sign.c_str());
    USBSerial.printf("request body: %s\n", body.c_str());
    http.addHeader("Content-Type", "application/json");
    http.addHeader("x-api-key", api_key);
    int responseCode = http.POST(body);
    String result_body = http.getString();
    http.end();
    USBSerial.printf("** POST result **\n");
    USBSerial.printf("responseCode: %d\n", responseCode);
    USBSerial.println("result_body: " + result_body);

    if (responseCode != 200)
    {
        return std::nullopt;
    }
    else
    {
        body.replace("\\\"", "\"");
        USBSerial.println("body: " + result_body);
        return result_body;
    }
}

std::optional<String> get_sesami_status(String device_uuid, String api_key)
{
    HTTPClient http;

    String url = String("https://app.candyhouse.co/api/sesame2/") + device_uuid;

    if (!http.begin(url))
    {
        return std::nullopt;
    }

    USBSerial.println("=== HTTPClient begin! ===");
    USBSerial.printf("URL: %s\n", url.c_str());
    http.addHeader("Content-Type", "application/json");
    http.addHeader("x-api-key", api_key);
    int responseCode = http.GET();
    String body = http.getString();
    USBSerial.printf("** GET status result **\n");
    USBSerial.printf("responseCode: %d\n", responseCode);
    body.replace("\\\"", "\"");
    USBSerial.println("body: " + body);
    http.end();

    return body;
}

std::optional<String> get_sesami_history(String device_uuid, String api_key)
{
    HTTPClient http;

    String url = String("https://app.candyhouse.co/api/sesame2/") + device_uuid + String("/history?page=0&lg=5");

    if (!http.begin(url))
    {
        return std::nullopt;
    }

    USBSerial.println("=== HTTPClient begin! ===");
    USBSerial.printf("URL: %s\n", url.c_str());
    http.addHeader("Content-Type", "application/json");
    http.addHeader("x-api-key", api_key);
    int responseCode = http.GET();
    String body = http.getString();
    USBSerial.printf("** GET history result **\n");
    USBSerial.printf("responseCode: %d\n", responseCode);
    body.replace("\\\"", "\"");
    USBSerial.println("body: " + body);
    http.end();

    return body;
}

#endif // SESAMI_UTIL_H