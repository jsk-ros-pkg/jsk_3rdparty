#ifndef SMART_DEVICE_PROTOCOL_PACKET_CREATOR_H__
#define SMART_DEVICE_PROTOCOL_PACKET_CREATOR_H__

#include <variant>
#include <vector>
#include <string.h>

#include <smart_device_protocol/Packet.h>

#include "sdp/packet_util.h"

void generate_meta_frame(uint8_t *packet, const char *device_name, const char *packet_description_01,
                         const char *serialization_format_01, const char *packet_description_02,
                         const char *serialization_format_02, const char *packet_description_03,
                         const char *serialization_format_03)
{
  *(uint16_t *)(packet + 0) = smart_device_protocol::Packet::PACKET_TYPE_META;
  strncpy((char *)(packet + 2), device_name, 20);
  strncpy((char *)(packet + 2 + 20), packet_description_01, 64);
  strncpy((char *)(packet + 2 + 20 + 64), serialization_format_01, 10);
  strncpy((char *)(packet + 2 + 20 + 64 + 10), packet_description_02, 64);
  strncpy((char *)(packet + 2 + 20 + 64 + 10 + 64), serialization_format_02, 10);
  strncpy((char *)(packet + 2 + 20 + 64 + 10 + 64 + 10), packet_description_03, 64);
  strncpy((char *)(packet + 2 + 20 + 64 + 10 + 64 + 10 + 64), serialization_format_03, 10);
}

bool generate_data_frame(uint8_t *packet, const char *packet_description, const char *serialization_format,
                         std::vector<SDPData> &data)
{
  if (not is_consistent_serialization_format(serialization_format, data))
  {
    return false;
  }
  if (strlen(serialization_format) != std::vector<SDPData>(data).size())
  {
    return false;
  }
  *(uint16_t *)(packet + 0) = smart_device_protocol::Packet::PACKET_TYPE_DATA;
  strncpy((char *)(packet + 2), packet_description, 64);
  strncpy((char *)(packet + 2 + 64), serialization_format, 10);
  auto packet_data_p = packet + 2 + 64 + 10;
  auto it = data.begin();
  int index_sf = 0;
  while (it != data.end())
  {
    std::string str;
    switch (serialization_format[index_sf])
    {
    case 'i': // std::holds_alternative<int32_t>(*it)
      *(int32_t *)packet_data_p = std::get<int32_t>(*it);
      packet_data_p += sizeof(int32_t);
      break;
    case 'f': // std::holds_alternative<float>(*it)
      *(float *)packet_data_p = std::get<float>(*it);
      packet_data_p += sizeof(float);
      break;
    case 'S': // std::holds_alternative<std::string>(*it)
      str = std::get<std::string>(*it);
      if (str.size() > 64)
      {
        str.resize(64);
      }
      for (int i = str.size(); i < 64; ++i)
      {
        *(char *)(packet_data_p + i) = '\0';
      }
      strncpy((char *)packet_data_p, str.c_str(), 64);
      packet_data_p += 64;
      break;
    case 's': // std::holds_alternative<std::string>(*it)
      str = std::get<std::string>(*it);
      if (str.size() > 16)
      {
        str.resize(64);
      }
      for (int i = str.size(); i < 16; ++i)
      {
        *(char *)(packet_data_p + i) = '\0';
      }
      strncpy((char *)packet_data_p, str.c_str(), str.size());
      packet_data_p += 16;
      break;
    case '?': // std::holds_alternative<bool>(*it)
    case 'b':
      *(bool *)packet_data_p = std::get<bool>(*it);
      packet_data_p += sizeof(bool);
      break;
    }
    ++it;
    ++index_sf;
  }
  return true;
}

bool generate_data_frame(uint8_t *packet, const char *packet_description,
                         std::vector<SDPData> &data)
{
  std::string serialization_format = get_serialization_format(data);
  return generate_data_frame(packet, packet_description, serialization_format.c_str(), data);
}

/* Version 1 functions */
void create_sensor_enviii_packet(uint8_t *packet, const char *module_name, int32_t pressure)
{
  *(uint16_t *)(packet + 0) = smart_device_protocol::Packet::PACKET_TYPE_SENSOR_ENV_III;
  strncpy((char *)(packet + 2), module_name, 64);
  *(int32_t *)(packet + 2 + 64) = pressure;
}

void create_sensor_stickv2_packet(uint8_t *packet, uint32_t number_of_person, const char *place_name)
{
  *(uint16_t *)(packet + 0) = smart_device_protocol::Packet::PACKET_TYPE_SENSOR_UNITV2_PERSON_COUNTER;
  *(uint32_t *)(packet + 2) = number_of_person;
  strncpy((char *)(packet + 2 + 4), place_name, 64);
}

void create_device_message_board_meta_packet(uint8_t *packet, const char *module_name)
{
  *(uint16_t *)(packet + 0) = smart_device_protocol::Packet::PACKET_TYPE_DEVICE_MESSAGE_BOARD_META;
  strncpy((char *)(packet + 2), module_name, 64);
}

void create_device_message_board_data_packet(uint8_t *packet, const char *source_name, uint64_t timeout_duration,
                                             const char *message)
{
  *(uint16_t *)(packet + 0) = smart_device_protocol::Packet::PACKET_TYPE_DEVICE_MESSAGE_BOARD_DATA;
  strncpy((char *)(packet + 2), source_name, 64);
  *(uint64_t *)(packet + 2 + 64) = timeout_duration;
  strncpy((char *)(packet + 2 + 64 + 8), message, 64);
}

#endif // SMART_DEVICE_PROTOCOL_PACKET_CREATOR_H__