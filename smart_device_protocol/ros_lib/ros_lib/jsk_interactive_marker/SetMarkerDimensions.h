#ifndef _ROS_SERVICE_SetMarkerDimensions_h
#define _ROS_SERVICE_SetMarkerDimensions_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "jsk_interactive_marker/MarkerDimensions.h"

namespace jsk_interactive_marker
{

static const char SETMARKERDIMENSIONS[] = "jsk_interactive_marker/SetMarkerDimensions";

  class SetMarkerDimensionsRequest : public ros::Msg
  {
    public:
      typedef const char* _target_name_type;
      _target_name_type target_name;
      typedef jsk_interactive_marker::MarkerDimensions _dimensions_type;
      _dimensions_type dimensions;

    SetMarkerDimensionsRequest():
      target_name(""),
      dimensions()
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
      offset += this->dimensions.serialize(outbuffer + offset);
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
      offset += this->dimensions.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SETMARKERDIMENSIONS; };
    virtual const char * getMD5() override { return "68f212be16364271f11f516c3f033749"; };

  };

  class SetMarkerDimensionsResponse : public ros::Msg
  {
    public:

    SetMarkerDimensionsResponse()
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

    virtual const char * getType() override { return SETMARKERDIMENSIONS; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetMarkerDimensions {
    public:
    typedef SetMarkerDimensionsRequest Request;
    typedef SetMarkerDimensionsResponse Response;
  };

}
#endif
