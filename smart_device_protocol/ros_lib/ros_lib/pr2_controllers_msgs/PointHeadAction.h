#ifndef _ROS_pr2_controllers_msgs_PointHeadAction_h
#define _ROS_pr2_controllers_msgs_PointHeadAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr2_controllers_msgs/PointHeadActionGoal.h"
#include "pr2_controllers_msgs/PointHeadActionResult.h"
#include "pr2_controllers_msgs/PointHeadActionFeedback.h"

namespace pr2_controllers_msgs
{

  class PointHeadAction : public ros::Msg
  {
    public:
      typedef pr2_controllers_msgs::PointHeadActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef pr2_controllers_msgs::PointHeadActionResult _action_result_type;
      _action_result_type action_result;
      typedef pr2_controllers_msgs::PointHeadActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    PointHeadAction():
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

    virtual const char * getType() override { return "pr2_controllers_msgs/PointHeadAction"; };
    virtual const char * getMD5() override { return "7252920f1243de1b741f14f214125371"; };

  };

}
#endif
