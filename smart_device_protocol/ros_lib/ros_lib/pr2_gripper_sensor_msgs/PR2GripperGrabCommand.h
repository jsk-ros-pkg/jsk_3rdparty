#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperGrabCommand_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperGrabCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperGrabCommand : public ros::Msg
  {
    public:
      typedef float _hardness_gain_type;
      _hardness_gain_type hardness_gain;

    PR2GripperGrabCommand():
      hardness_gain(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->hardness_gain);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->hardness_gain));
     return offset;
    }

    virtual const char * getType() override { return "pr2_gripper_sensor_msgs/PR2GripperGrabCommand"; };
    virtual const char * getMD5() override { return "cf286b093615060c79527896d36bf694"; };

  };

}
#endif
