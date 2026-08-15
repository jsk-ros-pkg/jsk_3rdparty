#ifndef _ROS_pr2_common_action_msgs_ArmMoveIKActionGoal_h
#define _ROS_pr2_common_action_msgs_ArmMoveIKActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "pr2_common_action_msgs/ArmMoveIKGoal.h"

namespace pr2_common_action_msgs
{

  class ArmMoveIKActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef pr2_common_action_msgs::ArmMoveIKGoal _goal_type;
      _goal_type goal;

    ArmMoveIKActionGoal():
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

    virtual const char * getType() override { return "pr2_common_action_msgs/ArmMoveIKActionGoal"; };
    virtual const char * getMD5() override { return "3e40f142a36fa484e230f6135ba5b21b"; };

  };

}
#endif
