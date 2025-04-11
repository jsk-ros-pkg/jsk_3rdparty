#ifndef _ROS_opencv_apps_Size_h
#define _ROS_opencv_apps_Size_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace opencv_apps
{

  class Size : public ros::Msg
  {
    public:
      typedef float _width_type;
      _width_type width;
      typedef float _height_type;
      _height_type height;

    Size():
      width(0),
      height(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->width);
      offset += serializeAvrFloat64(outbuffer + offset, this->height);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->width));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->height));
     return offset;
    }

    virtual const char * getType() override { return "opencv_apps/Size"; };
    virtual const char * getMD5() override { return "f86522e647336fb10b55359fe003f673"; };

  };

}
#endif
