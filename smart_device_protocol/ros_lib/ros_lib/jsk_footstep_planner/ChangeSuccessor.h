#ifndef _ROS_SERVICE_ChangeSuccessor_h
#define _ROS_SERVICE_ChangeSuccessor_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace jsk_footstep_planner
{

static const char CHANGESUCCESSOR[] = "jsk_footstep_planner/ChangeSuccessor";

  class ChangeSuccessorRequest : public ros::Msg
  {
    public:
      typedef const char* _type_type;
      _type_type type;

    ChangeSuccessorRequest():
      type("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_type = strlen(this->type);
      varToArr(outbuffer + offset, length_type);
      offset += 4;
      memcpy(outbuffer + offset, this->type, length_type);
      offset += length_type;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_type;
      arrToVar(length_type, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_type-1]=0;
      this->type = (char *)(inbuffer + offset-1);
      offset += length_type;
     return offset;
    }

    virtual const char * getType() override { return CHANGESUCCESSOR; };
    virtual const char * getMD5() override { return "dc67331de85cf97091b7d45e5c64ab75"; };

  };

  class ChangeSuccessorResponse : public ros::Msg
  {
    public:

    ChangeSuccessorResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return CHANGESUCCESSOR; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ChangeSuccessor {
    public:
    typedef ChangeSuccessorRequest Request;
    typedef ChangeSuccessorResponse Response;
  };

}
#endif
