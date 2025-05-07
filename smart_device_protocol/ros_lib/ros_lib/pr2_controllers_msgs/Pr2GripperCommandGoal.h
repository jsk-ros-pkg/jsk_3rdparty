#ifndef _ROS_pr2_controllers_msgs_Pr2GripperCommandGoal_h
#define _ROS_pr2_controllers_msgs_Pr2GripperCommandGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr2_controllers_msgs/Pr2GripperCommand.h"

namespace pr2_controllers_msgs
{

  class Pr2GripperCommandGoal : public ros::Msg
  {
    public:
      typedef pr2_controllers_msgs::Pr2GripperCommand _command_type;
      _command_type command;

    Pr2GripperCommandGoal():
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

    virtual const char * getType() override { return "pr2_controllers_msgs/Pr2GripperCommandGoal"; };
    virtual const char * getMD5() override { return "86fd82f4ddc48a4cb6856cfa69217e43"; };

  };

}
#endif
