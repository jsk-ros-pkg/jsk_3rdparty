#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperEventDetectorGoal_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperEventDetectorGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr2_gripper_sensor_msgs/PR2GripperEventDetectorCommand.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperEventDetectorGoal : public ros::Msg
  {
    public:
      typedef pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand _command_type;
      _command_type command;

    PR2GripperEventDetectorGoal():
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

    virtual const char * getType() override { return "pr2_gripper_sensor_msgs/PR2GripperEventDetectorGoal"; };
    virtual const char * getMD5() override { return "88b98e578eece7bef53cd48d37d3253b"; };

  };

}
#endif
