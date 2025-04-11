#ifndef _ROS_franka_msgs_ErrorRecoveryAction_h
#define _ROS_franka_msgs_ErrorRecoveryAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "franka_msgs/ErrorRecoveryActionGoal.h"
#include "franka_msgs/ErrorRecoveryActionResult.h"
#include "franka_msgs/ErrorRecoveryActionFeedback.h"

namespace franka_msgs
{

  class ErrorRecoveryAction : public ros::Msg
  {
    public:
      typedef franka_msgs::ErrorRecoveryActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef franka_msgs::ErrorRecoveryActionResult _action_result_type;
      _action_result_type action_result;
      typedef franka_msgs::ErrorRecoveryActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    ErrorRecoveryAction():
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

    virtual const char * getType() override { return "franka_msgs/ErrorRecoveryAction"; };
    virtual const char * getMD5() override { return "d5a016b49f278075666fbc901debbd08"; };

  };

}
#endif
