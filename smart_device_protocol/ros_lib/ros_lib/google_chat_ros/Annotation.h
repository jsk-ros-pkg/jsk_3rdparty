#ifndef _ROS_google_chat_ros_Annotation_h
#define _ROS_google_chat_ros_Annotation_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "google_chat_ros/User.h"
#include "google_chat_ros/SlashCommand.h"

namespace google_chat_ros
{

  class Annotation : public ros::Msg
  {
    public:
      typedef int32_t _length_type;
      _length_type length;
      typedef int32_t _start_index_type;
      _start_index_type start_index;
      typedef google_chat_ros::User _user_type;
      _user_type user;
      typedef bool _mention_type;
      _mention_type mention;
      typedef bool _slash_command_type;
      _slash_command_type slash_command;
      typedef google_chat_ros::SlashCommand _command_type;
      _command_type command;

    Annotation():
      length(0),
      start_index(0),
      user(),
      mention(0),
      slash_command(0),
      command()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_length;
      u_length.real = this->length;
      *(outbuffer + offset + 0) = (u_length.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_length.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_length.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_length.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->length);
      union {
        int32_t real;
        uint32_t base;
      } u_start_index;
      u_start_index.real = this->start_index;
      *(outbuffer + offset + 0) = (u_start_index.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_start_index.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_start_index.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_start_index.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->start_index);
      offset += this->user.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_mention;
      u_mention.real = this->mention;
      *(outbuffer + offset + 0) = (u_mention.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mention);
      union {
        bool real;
        uint8_t base;
      } u_slash_command;
      u_slash_command.real = this->slash_command;
      *(outbuffer + offset + 0) = (u_slash_command.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->slash_command);
      offset += this->command.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_length;
      u_length.base = 0;
      u_length.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_length.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_length.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_length.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->length = u_length.real;
      offset += sizeof(this->length);
      union {
        int32_t real;
        uint32_t base;
      } u_start_index;
      u_start_index.base = 0;
      u_start_index.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_start_index.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_start_index.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_start_index.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->start_index = u_start_index.real;
      offset += sizeof(this->start_index);
      offset += this->user.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_mention;
      u_mention.base = 0;
      u_mention.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->mention = u_mention.real;
      offset += sizeof(this->mention);
      union {
        bool real;
        uint8_t base;
      } u_slash_command;
      u_slash_command.base = 0;
      u_slash_command.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->slash_command = u_slash_command.real;
      offset += sizeof(this->slash_command);
      offset += this->command.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "google_chat_ros/Annotation"; };
    virtual const char * getMD5() override { return "802d9766a400b0c3a17014460dadcf6a"; };

  };

}
#endif
