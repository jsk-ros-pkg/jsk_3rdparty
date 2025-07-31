#ifndef _ROS_twist_mux_msgs_JoyPriorityAction_h
#define _ROS_twist_mux_msgs_JoyPriorityAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "twist_mux_msgs/JoyPriorityActionGoal.h"
#include "twist_mux_msgs/JoyPriorityActionResult.h"
#include "twist_mux_msgs/JoyPriorityActionFeedback.h"

namespace twist_mux_msgs
{

  class JoyPriorityAction : public ros::Msg
  {
    public:
      typedef twist_mux_msgs::JoyPriorityActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef twist_mux_msgs::JoyPriorityActionResult _action_result_type;
      _action_result_type action_result;
      typedef twist_mux_msgs::JoyPriorityActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    JoyPriorityAction():
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

    virtual const char * getType() override { return "twist_mux_msgs/JoyPriorityAction"; };
    virtual const char * getMD5() override { return "d5a016b49f278075666fbc901debbd08"; };

  };

}
#endif
