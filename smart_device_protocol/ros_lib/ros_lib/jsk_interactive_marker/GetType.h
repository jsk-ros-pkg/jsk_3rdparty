#ifndef _ROS_SERVICE_GetType_h
#define _ROS_SERVICE_GetType_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace jsk_interactive_marker
{

static const char GETTYPE[] = "jsk_interactive_marker/GetType";

  class GetTypeRequest : public ros::Msg
  {
    public:
      typedef const char* _target_name_type;
      _target_name_type target_name;

    GetTypeRequest():
      target_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_target_name = strlen(this->target_name);
      varToArr(outbuffer + offset, length_target_name);
      offset += 4;
      memcpy(outbuffer + offset, this->target_name, length_target_name);
      offset += length_target_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_target_name;
      arrToVar(length_target_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_target_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_target_name-1]=0;
      this->target_name = (char *)(inbuffer + offset-1);
      offset += length_target_name;
     return offset;
    }

    virtual const char * getType() override { return GETTYPE; };
    virtual const char * getMD5() override { return "2008933b3c7227647cbe00c74682331a"; };

  };

  class GetTypeResponse : public ros::Msg
  {
    public:
      typedef int32_t _type_type;
      _type_type type;

    GetTypeResponse():
      type(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_type;
      u_type.real = this->type;
      *(outbuffer + offset + 0) = (u_type.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_type.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_type.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_type.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->type);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_type;
      u_type.base = 0;
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->type = u_type.real;
      offset += sizeof(this->type);
     return offset;
    }

    virtual const char * getType() override { return GETTYPE; };
    virtual const char * getMD5() override { return "bda37decd5e3814bcc042f341d2e60a1"; };

  };

  class GetType {
    public:
    typedef GetTypeRequest Request;
    typedef GetTypeResponse Response;
  };

}
#endif
