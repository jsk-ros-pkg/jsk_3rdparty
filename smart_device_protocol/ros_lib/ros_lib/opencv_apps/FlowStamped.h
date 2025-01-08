#ifndef _ROS_opencv_apps_FlowStamped_h
#define _ROS_opencv_apps_FlowStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "opencv_apps/Flow.h"

namespace opencv_apps
{

  class FlowStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef opencv_apps::Flow _flow_type;
      _flow_type flow;

    FlowStamped():
      header(),
      flow()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->flow.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->flow.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "opencv_apps/FlowStamped"; };
    virtual const char * getMD5() override { return "b55faf909449963372b92417925b68cc"; };

  };

}
#endif
