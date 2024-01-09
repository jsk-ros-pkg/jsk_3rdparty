#ifndef _ROS_SERVICE_SetTransformableMarkerColor_h
#define _ROS_SERVICE_SetTransformableMarkerColor_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/ColorRGBA.h"

namespace jsk_interactive_marker
{

static const char SETTRANSFORMABLEMARKERCOLOR[] = "jsk_interactive_marker/SetTransformableMarkerColor";

  class SetTransformableMarkerColorRequest : public ros::Msg
  {
    public:
      typedef const char* _target_name_type;
      _target_name_type target_name;
      typedef std_msgs::ColorRGBA _color_type;
      _color_type color;

    SetTransformableMarkerColorRequest():
      target_name(""),
      color()
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
      offset += this->color.serialize(outbuffer + offset);
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
      offset += this->color.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SETTRANSFORMABLEMARKERCOLOR; };
    virtual const char * getMD5() override { return "6da9e77546dd19426d1dc251fb18d20e"; };

  };

  class SetTransformableMarkerColorResponse : public ros::Msg
  {
    public:

    SetTransformableMarkerColorResponse()
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

    virtual const char * getType() override { return SETTRANSFORMABLEMARKERCOLOR; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetTransformableMarkerColor {
    public:
    typedef SetTransformableMarkerColorRequest Request;
    typedef SetTransformableMarkerColorResponse Response;
  };

}
#endif
