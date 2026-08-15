#ifndef _ROS_naoqi_bridge_msgs_JointAnglesWithSpeedResult_h
#define _ROS_naoqi_bridge_msgs_JointAnglesWithSpeedResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/JointState.h"

namespace naoqi_bridge_msgs
{

  class JointAnglesWithSpeedResult : public ros::Msg
  {
    public:
      typedef sensor_msgs::JointState _goal_position_type;
      _goal_position_type goal_position;

    JointAnglesWithSpeedResult():
      goal_position()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->goal_position.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->goal_position.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "naoqi_bridge_msgs/JointAnglesWithSpeedResult"; };
    virtual const char * getMD5() override { return "1c77b3d9dc137611510fd16c3b792046"; };

  };

}
#endif
