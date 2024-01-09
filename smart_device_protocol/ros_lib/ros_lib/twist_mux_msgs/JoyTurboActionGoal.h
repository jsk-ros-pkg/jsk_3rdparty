#ifndef _ROS_twist_mux_msgs_JoyTurboActionGoal_h
#define _ROS_twist_mux_msgs_JoyTurboActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "twist_mux_msgs/JoyTurboGoal.h"

namespace twist_mux_msgs
{

  class JoyTurboActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef twist_mux_msgs::JoyTurboGoal _goal_type;
      _goal_type goal;

    JoyTurboActionGoal():
      header(),
      goal_id(),
      goal()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->goal_id.serialize(outbuffer + offset);
      offset += this->goal.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->goal_id.deserialize(inbuffer + offset);
      offset += this->goal.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "twist_mux_msgs/JoyTurboActionGoal"; };
    virtual const char * getMD5() override { return "4b30be6cd12b9e72826df56b481f40e0"; };

  };

}
#endif
