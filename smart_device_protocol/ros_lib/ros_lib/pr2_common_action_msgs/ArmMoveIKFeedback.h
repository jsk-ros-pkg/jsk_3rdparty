#ifndef _ROS_pr2_common_action_msgs_ArmMoveIKFeedback_h
#define _ROS_pr2_common_action_msgs_ArmMoveIKFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr2_common_action_msgs
{

  class ArmMoveIKFeedback : public ros::Msg
  {
    public:

    ArmMoveIKFeedback()
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

    virtual const char * getType() override { return "pr2_common_action_msgs/ArmMoveIKFeedback"; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

}
#endif
