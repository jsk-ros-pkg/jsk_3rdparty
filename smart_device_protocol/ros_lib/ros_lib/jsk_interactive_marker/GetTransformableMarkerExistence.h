#ifndef _ROS_SERVICE_GetTransformableMarkerExistence_h
#define _ROS_SERVICE_GetTransformableMarkerExistence_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace jsk_interactive_marker
{

static const char GETTRANSFORMABLEMARKEREXISTENCE[] = "jsk_interactive_marker/GetTransformableMarkerExistence";

  class GetTransformableMarkerExistenceRequest : public ros::Msg
  {
    public:
      typedef const char* _target_name_type;
      _target_name_type target_name;

    GetTransformableMarkerExistenceRequest():
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

    virtual const char * getType() override { return GETTRANSFORMABLEMARKEREXISTENCE; };
    virtual const char * getMD5() override { return "2008933b3c7227647cbe00c74682331a"; };

  };

  class GetTransformableMarkerExistenceResponse : public ros::Msg
  {
    public:
      typedef bool _existence_type;
      _existence_type existence;

    GetTransformableMarkerExistenceResponse():
      existence(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_existence;
      u_existence.real = this->existence;
      *(outbuffer + offset + 0) = (u_existence.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->existence);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_existence;
      u_existence.base = 0;
      u_existence.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->existence = u_existence.real;
      offset += sizeof(this->existence);
     return offset;
    }

    virtual const char * getType() override { return GETTRANSFORMABLEMARKEREXISTENCE; };
    virtual const char * getMD5() override { return "09ae207b2bf8af13a88dd5fd7b14cb66"; };

  };

  class GetTransformableMarkerExistence {
    public:
    typedef GetTransformableMarkerExistenceRequest Request;
    typedef GetTransformableMarkerExistenceResponse Response;
  };

}
#endif
