#ifndef _ROS_SERVICE_GetParam_h
#define _ROS_SERVICE_GetParam_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mongodb_store
{

static const char GETPARAM[] = "mongodb_store/GetParam";

  class GetParamRequest : public ros::Msg
  {
    public:
      typedef const char* _param_name_type;
      _param_name_type param_name;

    GetParamRequest():
      param_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_param_name = strlen(this->param_name);
      varToArr(outbuffer + offset, length_param_name);
      offset += 4;
      memcpy(outbuffer + offset, this->param_name, length_param_name);
      offset += length_param_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_param_name;
      arrToVar(length_param_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_param_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_param_name-1]=0;
      this->param_name = (char *)(inbuffer + offset-1);
      offset += length_param_name;
     return offset;
    }

    virtual const char * getType() override { return GETPARAM; };
    virtual const char * getMD5() override { return "b381fd4edcffd7ff5b5a7e1630491a17"; };

  };

  class GetParamResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _param_value_type;
      _param_value_type param_value;

    GetParamResponse():
      success(0),
      param_value("")
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
      uint32_t length_param_value = strlen(this->param_value);
      varToArr(outbuffer + offset, length_param_value);
      offset += 4;
      memcpy(outbuffer + offset, this->param_value, length_param_value);
      offset += length_param_value;
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
      uint32_t length_param_value;
      arrToVar(length_param_value, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_param_value; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_param_value-1]=0;
      this->param_value = (char *)(inbuffer + offset-1);
      offset += length_param_value;
     return offset;
    }

    virtual const char * getType() override { return GETPARAM; };
    virtual const char * getMD5() override { return "bfcec4af20d6b267ef6ee8d3934547c3"; };

  };

  class GetParam {
    public:
    typedef GetParamRequest Request;
    typedef GetParamResponse Response;
  };

}
#endif
