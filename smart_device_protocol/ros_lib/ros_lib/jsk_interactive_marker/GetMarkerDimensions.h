#ifndef _ROS_SERVICE_GetMarkerDimensions_h
#define _ROS_SERVICE_GetMarkerDimensions_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "jsk_interactive_marker/MarkerDimensions.h"

namespace jsk_interactive_marker
{

static const char GETMARKERDIMENSIONS[] = "jsk_interactive_marker/GetMarkerDimensions";

  class GetMarkerDimensionsRequest : public ros::Msg
  {
    public:
      typedef const char* _target_name_type;
      _target_name_type target_name;

    GetMarkerDimensionsRequest():
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

    virtual const char * getType() override { return GETMARKERDIMENSIONS; };
    virtual const char * getMD5() override { return "2008933b3c7227647cbe00c74682331a"; };

  };

  class GetMarkerDimensionsResponse : public ros::Msg
  {
    public:
      typedef jsk_interactive_marker::MarkerDimensions _dimensions_type;
      _dimensions_type dimensions;

    GetMarkerDimensionsResponse():
      dimensions()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->dimensions.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->dimensions.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return GETMARKERDIMENSIONS; };
    virtual const char * getMD5() override { return "742fb7645b415cf6f633f4bd78cac455"; };

  };

  class GetMarkerDimensions {
    public:
    typedef GetMarkerDimensionsRequest Request;
    typedef GetMarkerDimensionsResponse Response;
  };

}
#endif
