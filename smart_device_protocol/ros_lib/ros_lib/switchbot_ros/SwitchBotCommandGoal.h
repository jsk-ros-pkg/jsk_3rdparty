#ifndef _ROS_switchbot_ros_SwitchBotCommandGoal_h
#define _ROS_switchbot_ros_SwitchBotCommandGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace switchbot_ros
{

  class SwitchBotCommandGoal : public ros::Msg
  {
    public:
      typedef const char* _device_name_type;
      _device_name_type device_name;
      typedef const char* _command_type;
      _command_type command;
      typedef const char* _parameter_type;
      _parameter_type parameter;
      typedef const char* _command_type_type;
      _command_type_type command_type;

    SwitchBotCommandGoal():
      device_name(""),
      command(""),
      parameter(""),
      command_type("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_device_name = strlen(this->device_name);
      varToArr(outbuffer + offset, length_device_name);
      offset += 4;
      memcpy(outbuffer + offset, this->device_name, length_device_name);
      offset += length_device_name;
      uint32_t length_command = strlen(this->command);
      varToArr(outbuffer + offset, length_command);
      offset += 4;
      memcpy(outbuffer + offset, this->command, length_command);
      offset += length_command;
      uint32_t length_parameter = strlen(this->parameter);
      varToArr(outbuffer + offset, length_parameter);
      offset += 4;
      memcpy(outbuffer + offset, this->parameter, length_parameter);
      offset += length_parameter;
      uint32_t length_command_type = strlen(this->command_type);
      varToArr(outbuffer + offset, length_command_type);
      offset += 4;
      memcpy(outbuffer + offset, this->command_type, length_command_type);
      offset += length_command_type;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_device_name;
      arrToVar(length_device_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_device_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_device_name-1]=0;
      this->device_name = (char *)(inbuffer + offset-1);
      offset += length_device_name;
      uint32_t length_command;
      arrToVar(length_command, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_command; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_command-1]=0;
      this->command = (char *)(inbuffer + offset-1);
      offset += length_command;
      uint32_t length_parameter;
      arrToVar(length_parameter, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_parameter; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_parameter-1]=0;
      this->parameter = (char *)(inbuffer + offset-1);
      offset += length_parameter;
      uint32_t length_command_type;
      arrToVar(length_command_type, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_command_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_command_type-1]=0;
      this->command_type = (char *)(inbuffer + offset-1);
      offset += length_command_type;
     return offset;
    }

    virtual const char * getType() override { return "switchbot_ros/SwitchBotCommandGoal"; };
    virtual const char * getMD5() override { return "2c96c4dddec9221be5c5dc637b8e4f64"; };

  };

}
#endif
