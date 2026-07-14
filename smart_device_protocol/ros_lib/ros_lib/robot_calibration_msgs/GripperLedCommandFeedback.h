#ifndef _ROS_robot_calibration_msgs_GripperLedCommandFeedback_h
#define _ROS_robot_calibration_msgs_GripperLedCommandFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robot_calibration_msgs
{

  class GripperLedCommandFeedback : public ros::Msg
  {
    public:

    GripperLedCommandFeedback()
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

    virtual const char * getType() override { return "robot_calibration_msgs/GripperLedCommandFeedback"; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

}
#endif
