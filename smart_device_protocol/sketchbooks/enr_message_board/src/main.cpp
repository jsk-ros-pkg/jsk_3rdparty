
#include <vector>

#include <M5EPD.h>

#include <esp_system.h>
#include <esp_now.h>
#include <WiFi.h>

#include <smart_device_protocol/Packet.h>

#include "sdp/sdp.h"
#include "utils/config_loader.h"

#include "message.h"

// CONFIG
String device_name = "msg_board";

// CANVAS
M5EPD_Canvas canvas_title(&M5.EPD);
M5EPD_Canvas canvas_status(&M5.EPD);
M5EPD_Canvas canvas_message(&M5.EPD);

// SDP
uint8_t mac_address[6] = {0};

// MSG BUFFER
std::vector<Message> message_board;

// Others
int loop_counter = 0;

void OnDataRecvV1(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  uint8_t packet_type = get_packet_type(data);
  if (packet_type == smart_device_protocol::Packet::PACKET_TYPE_DEVICE_MESSAGE_BOARD_DATA)
  {
    auto m = Message(data);
    message_board.push_back(m);
    Serial.printf("Push message from V1 Data\n");
  }
}

void callback_for_v2(const uint8_t *mac_addr, const std::vector<SDPData> &body)
{
  std::string source_name = std::get<std::string>(body[0]);
  int32_t duration_until_deletion = std::get<int32_t>(body[1]);
  std::string message = std::get<std::string>(body[2]);

  auto m = Message((char *)source_name.c_str(), (char *)message.c_str(), duration_until_deletion);
  message_board.push_back(m);
  Serial.printf("Push message from V2 Data\n");
}

void load_config()
{
  StaticJsonDocument<1024> doc;
  if (not load_json_from_FS<1024>(SD, "/config.json", doc))
  {
    return;
  }
  if (doc.containsKey("device_name"))
    device_name = doc["device_name"].as<String>();
}

void clear_canvas(M5EPD_Canvas &canvas)
{
  canvas.clear();
  canvas.setCursor(0, 0);
}

void init_epd(M5EPD_Canvas &canvas_title, M5EPD_Canvas &canvas_status, M5EPD_Canvas &canvas_message)
{
  canvas_title.createCanvas(540, 100);
  canvas_status.createCanvas(540, 60);
  canvas_message.createCanvas(540, 800);
  canvas_title.setTextSize(3);
  canvas_status.setTextSize(2);
  canvas_message.setTextSize(2);
  clear_canvas(canvas_title);
  clear_canvas(canvas_status);
  clear_canvas(canvas_message);
}

void update_epd(M5EPD_Canvas &canvas_title, M5EPD_Canvas &canvas_status, M5EPD_Canvas &canvas_message)
{
  canvas_title.pushCanvas(0, 0, UPDATE_MODE_DU4);
  canvas_status.pushCanvas(0, 100, UPDATE_MODE_DU4);
  canvas_message.pushCanvas(0, 160, UPDATE_MODE_DU4);
}

void setup()
{
  esp_read_mac(mac_address, ESP_MAC_WIFI_STA);

  // Init M5Paper
  M5.begin(false, true, true, true, false);
  M5.EPD.SetRotation(90);
  M5.EPD.Clear(true);
  M5.RTC.begin();
  init_epd(canvas_title, canvas_status, canvas_message);
  Serial.println("Start init");

  // Load config
  load_config();

  // Init SDP
  init_sdp(mac_address, device_name);
  register_sdp_esp_now_recv_callback(OnDataRecvV1);
  register_sdp_interface_callback(Message::get_interface_description(), callback_for_v2);
  Serial.println("SDP Initialized!");

  // Show device info
  canvas_title.printf("ENR & SDP MESSAGE BOARD\n");
  canvas_title.printf("Name: %s\n", device_name.c_str());
  canvas_title.printf("ADDR: %2x:%2x:%2x:%2x:%2x:%2x\n",
                      mac_address[0], mac_address[1],
                      mac_address[2], mac_address[3],
                      mac_address[4], mac_address[5]);
  update_epd(canvas_title, canvas_status, canvas_message);
}

void loop()
{
  Serial.printf("Loop %d\n", loop_counter);
  uint8_t buf[250];

  // Manually Send V1 Meta Packet
  create_device_message_board_meta_packet(buf, device_name.c_str());
  broadcast_sdp_esp_now_packet((uint8_t *)buf, sizeof(buf));

  // Show Battery voltage
  uint32_t battery_voltage = M5.getBatteryVoltage();
  clear_canvas(canvas_status);
  if (loop_counter % 2 == 0)
  {
    canvas_status.printf("+ Battery: %u\n", battery_voltage);
  }
  else
  {
    canvas_status.printf("x Battery: %u\n", battery_voltage);
  }

  // Shoe messages
  clear_canvas(canvas_message);
  for (auto m = message_board.begin(); m != message_board.end();)
  {
    if (millis() > m->deadline)
    {
      m = message_board.erase(m);
    }
    else
    {
      m++;
    }
  }
  for (auto m = message_board.rbegin(); m != message_board.rend(); m++)
  {
    canvas_message.println("------------------------------------");
    canvas_message.printf("From: %s\n", m->source_name);
    canvas_message.printf("Duration until deletion(sec): %d\n", (int)((m->deadline - millis()) / 1000));
    canvas_message.printf("Message: %s\n\n", m->message);

    m->to_v1_packet(buf);
    broadcast_sdp_esp_now_packet((uint8_t *)buf, sizeof(buf));
    delay(10);
    m->to_v2_packet(buf);
    broadcast_sdp_esp_now_packet((uint8_t *)buf, sizeof(buf));
    delay(10);
  }
  update_epd(canvas_title, canvas_status, canvas_message);

  delay(100);
  loop_counter++;
}
