#ifndef _ROS_franka_msgs_ErrorRecoveryFeedback_h
#define _ROS_franka_msgs_ErrorRecoveryFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace franka_msgs
{

  class ErrorRecoveryFeedback : public ros::Msg
  {
    public:

    ErrorRecoveryFeedback()
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

    virtual const char * getType() override { return "franka_msgs/ErrorRecoveryFeedback"; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

}
#endif
