#ifndef _ROS_pr2_mechanism_msgs_SwitchControllerResult_h
#define _ROS_pr2_mechanism_msgs_SwitchControllerResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr2_mechanism_msgs
{

  class SwitchControllerResult : public ros::Msg
  {
    public:

    SwitchControllerResult()
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

    virtual const char * getType() override { return "pr2_mechanism_msgs/SwitchControllerResult"; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

}
#endif
