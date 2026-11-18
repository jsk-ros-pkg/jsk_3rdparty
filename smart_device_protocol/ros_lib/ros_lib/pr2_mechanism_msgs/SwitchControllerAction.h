#ifndef _ROS_pr2_mechanism_msgs_SwitchControllerAction_h
#define _ROS_pr2_mechanism_msgs_SwitchControllerAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr2_mechanism_msgs/SwitchControllerActionGoal.h"
#include "pr2_mechanism_msgs/SwitchControllerActionResult.h"
#include "pr2_mechanism_msgs/SwitchControllerActionFeedback.h"

namespace pr2_mechanism_msgs
{

  class SwitchControllerAction : public ros::Msg
  {
    public:
      typedef pr2_mechanism_msgs::SwitchControllerActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef pr2_mechanism_msgs::SwitchControllerActionResult _action_result_type;
      _action_result_type action_result;
      typedef pr2_mechanism_msgs::SwitchControllerActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    SwitchControllerAction():
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

    virtual const char * getType() override { return "pr2_mechanism_msgs/SwitchControllerAction"; };
    virtual const char * getMD5() override { return "c7b048ee44f1abe19d1dfae77332d13a"; };

  };

}
#endif
