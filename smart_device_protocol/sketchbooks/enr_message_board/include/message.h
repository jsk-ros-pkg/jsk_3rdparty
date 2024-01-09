#include <variant>
#include <vector>

#include <smart_device_protocol/Packet.h>

#include "sdp/packet_creator.h"
#include "sdp/packet_parser.h"

class Message
{
public:
  inline static std::string packet_description_write = "Message Board to write";
  inline static std::string serialization_format_write = "siS";
  inline static std::string packet_description_message = "Message Board message";

  char message[64];
  char source_name[64];
  unsigned long deadline;

  Message(const uint8_t *data)
  {

    strncpy(this->source_name, "None", 16);
    strncpy(this->source_name, "None", 64);
    this->deadline = 0;

    if (get_packet_type(data) == smart_device_protocol::Packet::PACKET_TYPE_DEVICE_MESSAGE_BOARD_DATA)
    {
      uint16_t packet_type;
      uint64_t timeout_duration;
      parse_packet_as_message_board_data_packet(data, packet_type, source_name, timeout_duration, message);
      this->deadline = millis() + timeout_duration;
    }
    else if (get_packet_type(data) == smart_device_protocol::Packet::PACKET_TYPE_DATA)
    {
      auto ret = parse_packet_as_data_packet(data);
      SDPInterfaceDescription packet_description_and_serialization_format = std::get<0>(ret);
      std::string packet_description = std::get<0>(packet_description_and_serialization_format);
      std::string serialization_format = std::get<1>(packet_description_and_serialization_format);
      std::vector<SDPData> data = std::get<1>(ret);

      if (packet_description == packet_description_write and serialization_format == serialization_format_write and get_serialization_format(data) == serialization_format_write)
      {
        std::string source_name = std::get<std::string>(data[0]);
        int32_t duration_until_deletion = std::get<int32_t>(data[1]);
        std::string message = std::get<std::string>(data[2]);
        strncpy(this->source_name, source_name.c_str(), 16);
        strncpy(this->message, message.c_str(), 64);
        this->deadline = millis() + duration_until_deletion;
      }
    }
  }

  Message(char *source_name, char *message, int32_t timeout_duration)
  {
    strncpy(this->source_name, source_name, 16);
    strncpy(this->message, message, 64);
    this->deadline = (int32_t)millis() + timeout_duration;
  }

  void to_v1_packet(uint8_t *data)
  {
    create_device_message_board_data_packet(data, source_name, deadline - millis(), message);
  }

  void to_v2_packet(uint8_t *data)
  {
    std::vector<std::variant<int32_t, float, std::string, bool>> data_vector;
    data_vector.push_back(std::variant<int32_t, float, std::string, bool>(std::string(source_name)));
    data_vector.push_back(std::variant<int32_t, float, std::string, bool>((int32_t)(this->deadline - millis())));
    data_vector.push_back(std::variant<int32_t, float, std::string, bool>(std::string(message)));
    generate_data_frame(data, packet_description_message.c_str(), data_vector);
  }

  static void generate_v2_meta_packet(uint8_t *data, const char *device_name)
  {
    generate_meta_frame(data, device_name, packet_description_write.c_str(), serialization_format_write.c_str(), "", "", "", "");
  }

  static SDPInterfaceDescription get_interface_description()
  {
    return std::make_tuple(packet_description_message, serialization_format_write);
  }
};
