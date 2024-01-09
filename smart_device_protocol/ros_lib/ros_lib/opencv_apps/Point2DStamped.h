#ifndef _ROS_opencv_apps_Point2DStamped_h
#define _ROS_opencv_apps_Point2DStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "opencv_apps/Point2D.h"

namespace opencv_apps
{

  class Point2DStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef opencv_apps::Point2D _point_type;
      _point_type point;

    Point2DStamped():
      header(),
      point()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->point.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->point.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "opencv_apps/Point2DStamped"; };
    virtual const char * getMD5() override { return "9f7db918fde9989a73131d0d083d049d"; };

  };

}
#endif
