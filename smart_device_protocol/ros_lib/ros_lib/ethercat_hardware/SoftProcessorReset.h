#ifndef _ROS_SERVICE_SoftProcessorReset_h
#define _ROS_SERVICE_SoftProcessorReset_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ethercat_hardware
{

static const char SOFTPROCESSORRESET[] = "ethercat_hardware/SoftProcessorReset";

  class SoftProcessorResetRequest : public ros::Msg
  {
    public:
      typedef const char* _actuator_name_type;
      _actuator_name_type actuator_name;
      typedef const char* _processor_name_type;
      _processor_name_type processor_name;

    SoftProcessorResetRequest():
      actuator_name(""),
      processor_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_actuator_name = strlen(this->actuator_name);
      varToArr(outbuffer + offset, length_actuator_name);
      offset += 4;
      memcpy(outbuffer + offset, this->actuator_name, length_actuator_name);
      offset += length_actuator_name;
      uint32_t length_processor_name = strlen(this->processor_name);
      varToArr(outbuffer + offset, length_processor_name);
      offset += 4;
      memcpy(outbuffer + offset, this->processor_name, length_processor_name);
      offset += length_processor_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_actuator_name;
      arrToVar(length_actuator_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_actuator_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_actuator_name-1]=0;
      this->actuator_name = (char *)(inbuffer + offset-1);
      offset += length_actuator_name;
      uint32_t length_processor_name;
      arrToVar(length_processor_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_processor_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_processor_name-1]=0;
      this->processor_name = (char *)(inbuffer + offset-1);
      offset += length_processor_name;
     return offset;
    }

    virtual const char * getType() override { return SOFTPROCESSORRESET; };
    virtual const char * getMD5() override { return "777be25d71e9e85e62fa14223ffddb6b"; };

  };

  class SoftProcessorResetResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _error_msg_type;
      _error_msg_type error_msg;

    SoftProcessorResetResponse():
      success(0),
      error_msg("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_error_msg = strlen(this->error_msg);
      varToArr(outbuffer + offset, length_error_msg);
      offset += 4;
      memcpy(outbuffer + offset, this->error_msg, length_error_msg);
      offset += length_error_msg;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_error_msg;
      arrToVar(length_error_msg, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_error_msg; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_error_msg-1]=0;
      this->error_msg = (char *)(inbuffer + offset-1);
      offset += length_error_msg;
     return offset;
    }

    virtual const char * getType() override { return SOFTPROCESSORRESET; };
    virtual const char * getMD5() override { return "d006c48be24db1173a071ca9af4c8179"; };

  };

  class SoftProcessorReset {
    public:
    typedef SoftProcessorResetRequest Request;
    typedef SoftProcessorResetResponse Response;
  };

}
#endif
