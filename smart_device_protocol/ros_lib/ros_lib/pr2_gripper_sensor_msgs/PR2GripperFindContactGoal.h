#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperFindContactGoal_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperFindContactGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr2_gripper_sensor_msgs/PR2GripperFindContactCommand.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperFindContactGoal : public ros::Msg
  {
    public:
      typedef pr2_gripper_sensor_msgs::PR2GripperFindContactCommand _command_type;
      _command_type command;

    PR2GripperFindContactGoal():
      command()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->command.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->command.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "pr2_gripper_sensor_msgs/PR2GripperFindContactGoal"; };
    virtual const char * getMD5() override { return "f0ae570e217e7429eba0f205341933a0"; };

  };

}
#endif
