#ifndef _ROS_opencv_apps_Circle_h
#define _ROS_opencv_apps_Circle_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "opencv_apps/Point2D.h"

namespace opencv_apps
{

  class Circle : public ros::Msg
  {
    public:
      typedef opencv_apps::Point2D _center_type;
      _center_type center;
      typedef float _radius_type;
      _radius_type radius;

    Circle():
      center(),
      radius(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->center.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->radius);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->center.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->radius));
     return offset;
    }

    virtual const char * getType() override { return "opencv_apps/Circle"; };
    virtual const char * getMD5() override { return "4f6847051b4fe493b5af8caad66201d5"; };

  };

}
#endif
