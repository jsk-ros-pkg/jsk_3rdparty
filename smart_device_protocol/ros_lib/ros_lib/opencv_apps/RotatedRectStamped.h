#ifndef _ROS_opencv_apps_RotatedRectStamped_h
#define _ROS_opencv_apps_RotatedRectStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "opencv_apps/RotatedRect.h"

namespace opencv_apps
{

  class RotatedRectStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef opencv_apps::RotatedRect _rect_type;
      _rect_type rect;

    RotatedRectStamped():
      header(),
      rect()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->rect.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->rect.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "opencv_apps/RotatedRectStamped"; };
    virtual const char * getMD5() override { return "ba2d76a1968e4f77570c01223781fe15"; };

  };

}
#endif
