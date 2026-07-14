#ifndef SWITCHBOT_UTIL_H
#define SWITCHBOT_UTIL_H

#include <M5AtomS3.h>
#include <HTTPClient.h>

#include <mbedtls/md.h>

#include <base64.h>

#include <optional>

// Copied from https://qiita.com/poruruba/items/2ba3f1ff3e3045ce26c1
void hmac_sha256(const char *p_key, const char *p_payload, unsigned char *p_hmacResult)
{
    mbedtls_md_context_t ctx;
    mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;
    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 1);
    mbedtls_md_hmac_starts(&ctx, (const unsigned char *)p_key, strlen(p_key));
    mbedtls_md_hmac_update(&ctx, (const unsigned char *)p_payload, strlen(p_payload));
    mbedtls_md_hmac_finish(&ctx, p_hmacResult); // 32 bytes
    mbedtls_md_free(&ctx);
}

std::tuple<String, String, String> make_switchbot_sign(String &token, String &secret)
{
    String nonce = "";
    uint32_t t = (uint32_t)std::time(nullptr);
    String t_str = String(t) + "000";
    String sign_str = token + t_str + nonce;
    unsigned char hmacResult[32];
    hmac_sha256(secret.c_str(), sign_str.c_str(), hmacResult);
    String sign = base64::encode(hmacResult, 32);
    return std::make_tuple(sign, t_str, nonce);
}

std::optional<String> get_device_list(String &token, String &secret)
{
    HTTPClient http;
    String url = "https://api.switch-bot.com/v1.1/devices";
    String sign, t, nonce;
    std::tie(sign, t, nonce) = make_switchbot_sign(token, secret);
    if (!http.begin(url))
    {
        USBSerial.println("http.begin() failed");
        return std::nullopt;
    }
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", token);
    http.addHeader("sign", sign);
    http.addHeader("t", t);
    http.addHeader("nonce", nonce);
    int responseCode = http.GET();
    String result_body = http.getString();
    http.end();

    USBSerial.printf("** GET result **\n");
    USBSerial.printf("responseCode: %d\n", responseCode);
    USBSerial.println("body: " + result_body);

    if (responseCode != 200)
    {
        return std::nullopt;
    }
    else
    {
        return result_body;
    }
}

std::optional<String> get_device_status(String &token, String &secret, String &device_id)
{
    HTTPClient http;
    String url = "https://api.switch-bot.com/v1.1/devices/" + device_id + "/status";
    String sign, t, nonce;
    std::tie(sign, t, nonce) = make_switchbot_sign(token, secret);
    if (!http.begin(url))
    {
        USBSerial.println("http.begin() failed");
        return std::nullopt;
    }
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", token);
    http.addHeader("sign", sign);
    http.addHeader("t", t);
    http.addHeader("nonce", nonce);
    int responseCode = http.GET();
    String result_body = http.getString();
    http.end();

    USBSerial.printf("** GET result **\n");
    USBSerial.printf("responseCode: %d\n", responseCode);
    USBSerial.println("body: " + result_body);
    if (responseCode != 200)
    {
        return std::nullopt;
    }
    else
    {
        return result_body;
    }
}

std::optional<String> send_device_command(
    String &token,
    String &secret,
    String &device_id,
    String &command_type,
    String &command)
{
    HTTPClient http;
    String url = "https://api.switch-bot.com/v1.1/devices/" + device_id + "/commands";
    String sign, t, nonce;
    std::tie(sign, t, nonce) = make_switchbot_sign(token, secret);
    if (!http.begin(url))
    {
        USBSerial.println("http.begin() failed");
        return std::nullopt;
    }
    http.addHeader("Authorization", token);
    http.addHeader("sign", sign);
    http.addHeader("t", t);
    http.addHeader("nonce", nonce);
    String body = "{";
    body = body + "\"command\":\"" + command + "\",";
    body = body + "\"commandType\":\"" + command_type + "\",";
    body = body + "\"parameter\":\"" + "default" + "\"";
    body = body + "}";
    int responseCode = http.POST(body);
    String result_body = http.getString();
    http.end();

    USBSerial.printf("** POST result **\n");
    USBSerial.printf("responseCode: %d\n", responseCode);
    USBSerial.println("body: " + result_body);
    if (responseCode != 200)
    {
        return std::nullopt;
    }
    else
    {
        return result_body;
    }
}

#endif // SWITCHBOT_UTIL_H