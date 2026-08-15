#ifndef _ROS_naoqi_bridge_msgs_BodyPoseResult_h
#define _ROS_naoqi_bridge_msgs_BodyPoseResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace naoqi_bridge_msgs
{

  class BodyPoseResult : public ros::Msg
  {
    public:

    BodyPoseResult()
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

    virtual const char * getType() override { return "naoqi_bridge_msgs/BodyPoseResult"; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

}
#endif
