#ifndef _ROS_naoqi_bridge_msgs_JointTrajectoryGoal_h
#define _ROS_naoqi_bridge_msgs_JointTrajectoryGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "trajectory_msgs/JointTrajectory.h"

namespace naoqi_bridge_msgs
{

  class JointTrajectoryGoal : public ros::Msg
  {
    public:
      typedef trajectory_msgs::JointTrajectory _trajectory_type;
      _trajectory_type trajectory;
      typedef uint8_t _relative_type;
      _relative_type relative;

    JointTrajectoryGoal():
      trajectory(),
      relative(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->trajectory.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->relative >> (8 * 0)) & 0xFF;
      offset += sizeof(this->relative);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->trajectory.deserialize(inbuffer + offset);
      this->relative =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->relative);
     return offset;
    }

    virtual const char * getType() override { return "naoqi_bridge_msgs/JointTrajectoryGoal"; };
    virtual const char * getMD5() override { return "7ecdd56459ac4b8e2c210a74dbb66523"; };

  };

}
#endif
