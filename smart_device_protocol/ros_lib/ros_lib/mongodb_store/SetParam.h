#ifndef _ROS_SERVICE_SetParam_h
#define _ROS_SERVICE_SetParam_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mongodb_store
{

static const char SETPARAM[] = "mongodb_store/SetParam";

  class SetParamRequest : public ros::Msg
  {
    public:
      typedef const char* _param_type;
      _param_type param;

    SetParamRequest():
      param("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_param = strlen(this->param);
      varToArr(outbuffer + offset, length_param);
      offset += 4;
      memcpy(outbuffer + offset, this->param, length_param);
      offset += length_param;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_param;
      arrToVar(length_param, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_param; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_param-1]=0;
      this->param = (char *)(inbuffer + offset-1);
      offset += length_param;
     return offset;
    }

    virtual const char * getType() override { return SETPARAM; };
    virtual const char * getMD5() override { return "eb04b7504512676dca105ab8842899a4"; };

  };

  class SetParamResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetParamResponse():
      success(0)
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
     return offset;
    }

    virtual const char * getType() override { return SETPARAM; };
    virtual const char * getMD5() override { return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetParam {
    public:
    typedef SetParamRequest Request;
    typedef SetParamResponse Response;
  };

}
#endif
