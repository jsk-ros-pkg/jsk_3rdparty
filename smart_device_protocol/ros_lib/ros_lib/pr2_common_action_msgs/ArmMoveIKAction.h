#ifndef _ROS_pr2_common_action_msgs_ArmMoveIKAction_h
#define _ROS_pr2_common_action_msgs_ArmMoveIKAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr2_common_action_msgs/ArmMoveIKActionGoal.h"
#include "pr2_common_action_msgs/ArmMoveIKActionResult.h"
#include "pr2_common_action_msgs/ArmMoveIKActionFeedback.h"

namespace pr2_common_action_msgs
{

  class ArmMoveIKAction : public ros::Msg
  {
    public:
      typedef pr2_common_action_msgs::ArmMoveIKActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef pr2_common_action_msgs::ArmMoveIKActionResult _action_result_type;
      _action_result_type action_result;
      typedef pr2_common_action_msgs::ArmMoveIKActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    ArmMoveIKAction():
      action_goal(),
      action_result(),
      action_feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "pr2_common_action_msgs/ArmMoveIKAction"; };
    virtual const char * getMD5() override { return "f9df8066f1c6f2acf0247564f92e5ff2"; };

  };

}
#endif
