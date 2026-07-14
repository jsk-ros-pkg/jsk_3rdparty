#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperSlipServoGoal_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperSlipServoGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr2_gripper_sensor_msgs/PR2GripperSlipServoCommand.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperSlipServoGoal : public ros::Msg
  {
    public:
      typedef pr2_gripper_sensor_msgs::PR2GripperSlipServoCommand _command_type;
      _command_type command;

    PR2GripperSlipServoGoal():
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

    virtual const char * getType() override { return "pr2_gripper_sensor_msgs/PR2GripperSlipServoGoal"; };
    virtual const char * getMD5() override { return "bf76e656d304158c04ab279db7cefc85"; };

  };

}
#endif
