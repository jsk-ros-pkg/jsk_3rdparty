#ifndef _ROS_opencv_apps_RotatedRect_h
#define _ROS_opencv_apps_RotatedRect_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "opencv_apps/Point2D.h"
#include "opencv_apps/Size.h"

namespace opencv_apps
{

  class RotatedRect : public ros::Msg
  {
    public:
      typedef float _angle_type;
      _angle_type angle;
      typedef opencv_apps::Point2D _center_type;
      _center_type center;
      typedef opencv_apps::Size _size_type;
      _size_type size;

    RotatedRect():
      angle(0),
      center(),
      size()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->angle);
      offset += this->center.serialize(outbuffer + offset);
      offset += this->size.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->angle));
      offset += this->center.deserialize(inbuffer + offset);
      offset += this->size.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "opencv_apps/RotatedRect"; };
    virtual const char * getMD5() override { return "0ae60505c52f020176686d0689b8d390"; };

  };

}
#endif
