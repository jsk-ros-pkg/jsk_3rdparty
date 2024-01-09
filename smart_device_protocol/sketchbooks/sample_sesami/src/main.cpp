#include <M5AtomS3.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <base64.h>

#include "iot_com_util/Time.h"
#include "web_services/sesami_util.h"

const char *ssid = "";
const char *password = "";
WiFiMulti WiFiMulti;

String device_uuid = "";
String secret_key = "";
String api_key = "";

void setup()
{
    int sum = 0;
    M5.begin(true, true, false, false);
    WiFiMulti.addAP(ssid, password);
    M5.lcd.printf("Waiting connect to WiFi: %s ...", ssid);
    while (WiFiMulti.run() != WL_CONNECTED)
    {
        M5.lcd.print(".");
        delay(1000);
        sum += 1;
        if (sum == 8)
            M5.lcd.print("Conncet failed!");
    }
    M5.lcd.println("\nWiFi connected");
    M5.lcd.print("IP address: ");
    M5.lcd.println(WiFi.localIP());
    delay(500);

    Time.set_time();
}

void loop()
{
    delay(1000);
    get_sesami_status(
        device_uuid,
        api_key);
    USBSerial.println();

    delay(1000);
    get_sesami_history(
        device_uuid,
        api_key);
    USBSerial.println();

    delay(1000);
    operation_sesami(
        device_uuid,
        api_key,
        88,
        secret_key);
    USBSerial.println();
}
