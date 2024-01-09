#ifndef _ROS_pr2_common_action_msgs_TuckArmsAction_h
#define _ROS_pr2_common_action_msgs_TuckArmsAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr2_common_action_msgs/TuckArmsActionGoal.h"
#include "pr2_common_action_msgs/TuckArmsActionResult.h"
#include "pr2_common_action_msgs/TuckArmsActionFeedback.h"

namespace pr2_common_action_msgs
{

  class TuckArmsAction : public ros::Msg
  {
    public:
      typedef pr2_common_action_msgs::TuckArmsActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef pr2_common_action_msgs::TuckArmsActionResult _action_result_type;
      _action_result_type action_result;
      typedef pr2_common_action_msgs::TuckArmsActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    TuckArmsAction():
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

    virtual const char * getType() override { return "pr2_common_action_msgs/TuckArmsAction"; };
    virtual const char * getMD5() override { return "4385d436aaa7b0cb11299848e25300f5"; };

  };

}
#endif
