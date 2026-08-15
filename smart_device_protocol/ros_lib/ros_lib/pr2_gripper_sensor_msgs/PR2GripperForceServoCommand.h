#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperForceServoCommand_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperForceServoCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperForceServoCommand : public ros::Msg
  {
    public:
      typedef float _fingertip_force_type;
      _fingertip_force_type fingertip_force;

    PR2GripperForceServoCommand():
      fingertip_force(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->fingertip_force);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->fingertip_force));
     return offset;
    }

    virtual const char * getType() override { return "pr2_gripper_sensor_msgs/PR2GripperForceServoCommand"; };
    virtual const char * getMD5() override { return "dd4b2a0dfafa27b67d2002841f544379"; };

  };

}
#endif
