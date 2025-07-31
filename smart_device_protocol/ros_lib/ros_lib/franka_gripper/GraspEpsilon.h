#ifndef _ROS_franka_gripper_GraspEpsilon_h
#define _ROS_franka_gripper_GraspEpsilon_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace franka_gripper
{

  class GraspEpsilon : public ros::Msg
  {
    public:
      typedef float _inner_type;
      _inner_type inner;
      typedef float _outer_type;
      _outer_type outer;

    GraspEpsilon():
      inner(0),
      outer(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->inner);
      offset += serializeAvrFloat64(outbuffer + offset, this->outer);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->inner));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->outer));
     return offset;
    }

    virtual const char * getType() override { return "franka_gripper/GraspEpsilon"; };
    virtual const char * getMD5() override { return "95b2c5464a6f679bd1dacaf86414f095"; };

  };

}
#endif
