#ifndef _ROS_pr2_common_action_msgs_TuckArmsActionGoal_h
#define _ROS_pr2_common_action_msgs_TuckArmsActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "pr2_common_action_msgs/TuckArmsGoal.h"

namespace pr2_common_action_msgs
{

  class TuckArmsActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef pr2_common_action_msgs::TuckArmsGoal _goal_type;
      _goal_type goal;

    TuckArmsActionGoal():
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

    virtual const char * getType() override { return "pr2_common_action_msgs/TuckArmsActionGoal"; };
    virtual const char * getMD5() override { return "c8b69f27f8015edb9bf64bbf03a42618"; };

  };

}
#endif
