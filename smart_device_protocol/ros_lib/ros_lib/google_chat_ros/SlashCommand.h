#ifndef _ROS_google_chat_ros_SlashCommand_h
#define _ROS_google_chat_ros_SlashCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "google_chat_ros/User.h"

namespace google_chat_ros
{

  class SlashCommand : public ros::Msg
  {
    public:
      typedef google_chat_ros::User _user_type;
      _user_type user;
      typedef bool _added_type;
      _added_type added;
      typedef bool _invoke_type;
      _invoke_type invoke;
      typedef const char* _command_name_type;
      _command_name_type command_name;
      typedef const char* _command_id_type;
      _command_id_type command_id;
      typedef bool _triggers_dialog_type;
      _triggers_dialog_type triggers_dialog;

    SlashCommand():
      user(),
      added(0),
      invoke(0),
      command_name(""),
      command_id(""),
      triggers_dialog(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->user.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_added;
      u_added.real = this->added;
      *(outbuffer + offset + 0) = (u_added.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->added);
      union {
        bool real;
        uint8_t base;
      } u_invoke;
      u_invoke.real = this->invoke;
      *(outbuffer + offset + 0) = (u_invoke.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->invoke);
      uint32_t length_command_name = strlen(this->command_name);
      varToArr(outbuffer + offset, length_command_name);
      offset += 4;
      memcpy(outbuffer + offset, this->command_name, length_command_name);
      offset += length_command_name;
      uint32_t length_command_id = strlen(this->command_id);
      varToArr(outbuffer + offset, length_command_id);
      offset += 4;
      memcpy(outbuffer + offset, this->command_id, length_command_id);
      offset += length_command_id;
      union {
        bool real;
        uint8_t base;
      } u_triggers_dialog;
      u_triggers_dialog.real = this->triggers_dialog;
      *(outbuffer + offset + 0) = (u_triggers_dialog.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->triggers_dialog);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->user.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_added;
      u_added.base = 0;
      u_added.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->added = u_added.real;
      offset += sizeof(this->added);
      union {
        bool real;
        uint8_t base;
      } u_invoke;
      u_invoke.base = 0;
      u_invoke.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->invoke = u_invoke.real;
      offset += sizeof(this->invoke);
      uint32_t length_command_name;
      arrToVar(length_command_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_command_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_command_name-1]=0;
      this->command_name = (char *)(inbuffer + offset-1);
      offset += length_command_name;
      uint32_t length_command_id;
      arrToVar(length_command_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_command_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_command_id-1]=0;
      this->command_id = (char *)(inbuffer + offset-1);
      offset += length_command_id;
      union {
        bool real;
        uint8_t base;
      } u_triggers_dialog;
      u_triggers_dialog.base = 0;
      u_triggers_dialog.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->triggers_dialog = u_triggers_dialog.real;
      offset += sizeof(this->triggers_dialog);
     return offset;
    }

    virtual const char * getType() override { return "google_chat_ros/SlashCommand"; };
    virtual const char * getMD5() override { return "528efb48a0c7a11bda8f24aa72b809fb"; };

  };

}
#endif
