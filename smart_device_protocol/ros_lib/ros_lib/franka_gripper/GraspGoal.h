#ifndef _ROS_franka_gripper_GraspGoal_h
#define _ROS_franka_gripper_GraspGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "franka_gripper/GraspEpsilon.h"

namespace franka_gripper
{

  class GraspGoal : public ros::Msg
  {
    public:
      typedef float _width_type;
      _width_type width;
      typedef franka_gripper::GraspEpsilon _epsilon_type;
      _epsilon_type epsilon;
      typedef float _speed_type;
      _speed_type speed;
      typedef float _force_type;
      _force_type force;

    GraspGoal():
      width(0),
      epsilon(),
      speed(0),
      force(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->width);
      offset += this->epsilon.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->speed);
      offset += serializeAvrFloat64(outbuffer + offset, this->force);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->width));
      offset += this->epsilon.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->speed));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->force));
     return offset;
    }

    virtual const char * getType() override { return "franka_gripper/GraspGoal"; };
    virtual const char * getMD5() override { return "627a0f0b10ad0c919fbd62b0b3427e63"; };

  };

}
#endif
