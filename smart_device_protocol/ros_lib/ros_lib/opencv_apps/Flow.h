#ifndef _ROS_opencv_apps_Flow_h
#define _ROS_opencv_apps_Flow_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "opencv_apps/Point2D.h"

namespace opencv_apps
{

  class Flow : public ros::Msg
  {
    public:
      typedef opencv_apps::Point2D _point_type;
      _point_type point;
      typedef opencv_apps::Point2D _velocity_type;
      _velocity_type velocity;

    Flow():
      point(),
      velocity()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->point.serialize(outbuffer + offset);
      offset += this->velocity.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->point.deserialize(inbuffer + offset);
      offset += this->velocity.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "opencv_apps/Flow"; };
    virtual const char * getMD5() override { return "dd9a9efd88ba39035e78af697593d751"; };

  };

}
#endif
