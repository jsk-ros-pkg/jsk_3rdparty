// MIT License

// Copyright (c) 2020 M5Stack

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


// Mainly copied from
// https://github.com/m5stack/TimerCam-arduino/tree/0.0.2

#include "esp_camera.h"
#include "camera_pins.h"
#include "battery.h"

// Data structure of camera frame buffer
// https://github.com/espressif/esp32-camera/blob/7b6f020939be574b1da9d4668327321edefd4e8d/driver/include/esp_camera.h#L146-L156
camera_fb_t * fb = NULL;
static int64_t last_frame = 0;

uint32_t bat_voltage = 0;

void setupBattery() {
    bat_init();
}

void setupTimerCam() {
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    Serial.println();
    pinMode(2, OUTPUT);
    digitalWrite(2, HIGH);

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10; //10-63 lower number means higher quality
    config.fb_count = 2;

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }

    sensor_t * s = esp_camera_sensor_get();
    //initial sensors are flipped vertically and colors are a bit saturated
    // Change camera parameters based on your application
    // https://github.com/espressif/esp32-camera/blob/master/driver/include/sensor.h
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_saturation(s, -2);//lower the saturation

    //drop down frame size for higher initial frame rate
    // Note that each image size must be less than publisher output size (8192 byte),
    // which is defined at m5stack_ros.h
    // You can also change config.jpeg_quality
    // FYI: available formats
    // https://github.com/espressif/esp32-camera/blob/7b6f020939be574b1da9d4668327321edefd4e8d/driver/include/sensor.h#L81-L107
    s->set_framesize(s, FRAMESIZE_SVGA); // 800x600
}

void readBattery() {
    // You can use BASE_VOLATAGE 3600 (typo in the origin!) as threshold
    bat_voltage = bat_get_voltage();
}

static esp_err_t readTimerCam(){
    // Release fb defined in the last readTimerCam()
    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
    }

    esp_err_t res = ESP_OK;

    if(!last_frame) {
        last_frame = esp_timer_get_time();
    }

    fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        res = ESP_FAIL;
    }
    if(res != ESP_OK){
        Serial.println("Something wrong!");
    }
    // Calculation times
    int64_t fr_end = esp_timer_get_time();

    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;
    frame_time /= 1000;
    Serial.printf("MJPG: %uB %ums (%.1ffps)\r\n",
                  (uint32_t)(fb->len),
                  (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time
                  );

    return res;
}
