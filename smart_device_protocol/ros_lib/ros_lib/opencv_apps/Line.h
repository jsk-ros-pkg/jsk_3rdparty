#ifndef _ROS_opencv_apps_Line_h
#define _ROS_opencv_apps_Line_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "opencv_apps/Point2D.h"

namespace opencv_apps
{

  class Line : public ros::Msg
  {
    public:
      typedef opencv_apps::Point2D _pt1_type;
      _pt1_type pt1;
      typedef opencv_apps::Point2D _pt2_type;
      _pt2_type pt2;

    Line():
      pt1(),
      pt2()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->pt1.serialize(outbuffer + offset);
      offset += this->pt2.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->pt1.deserialize(inbuffer + offset);
      offset += this->pt2.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "opencv_apps/Line"; };
    virtual const char * getMD5() override { return "a1419010b3fc4549e3f450018363d000"; };

  };

}
#endif
