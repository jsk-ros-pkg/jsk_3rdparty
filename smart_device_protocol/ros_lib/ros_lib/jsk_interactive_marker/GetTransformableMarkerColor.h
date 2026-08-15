#ifndef _ROS_SERVICE_GetTransformableMarkerColor_h
#define _ROS_SERVICE_GetTransformableMarkerColor_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/ColorRGBA.h"

namespace jsk_interactive_marker
{

static const char GETTRANSFORMABLEMARKERCOLOR[] = "jsk_interactive_marker/GetTransformableMarkerColor";

  class GetTransformableMarkerColorRequest : public ros::Msg
  {
    public:
      typedef const char* _target_name_type;
      _target_name_type target_name;

    GetTransformableMarkerColorRequest():
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

    virtual const char * getType() override { return GETTRANSFORMABLEMARKERCOLOR; };
    virtual const char * getMD5() override { return "2008933b3c7227647cbe00c74682331a"; };

  };

  class GetTransformableMarkerColorResponse : public ros::Msg
  {
    public:
      typedef std_msgs::ColorRGBA _color_type;
      _color_type color;

    GetTransformableMarkerColorResponse():
      color()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->color.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->color.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return GETTRANSFORMABLEMARKERCOLOR; };
    virtual const char * getMD5() override { return "3e04b62b1b39cd97e873789f0bb130e7"; };

  };

  class GetTransformableMarkerColor {
    public:
    typedef GetTransformableMarkerColorRequest Request;
    typedef GetTransformableMarkerColorResponse Response;
  };

}
#endif
