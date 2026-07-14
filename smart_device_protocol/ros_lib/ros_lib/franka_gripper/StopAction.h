#ifndef _ROS_franka_gripper_StopAction_h
#define _ROS_franka_gripper_StopAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "franka_gripper/StopActionGoal.h"
#include "franka_gripper/StopActionResult.h"
#include "franka_gripper/StopActionFeedback.h"

namespace franka_gripper
{

  class StopAction : public ros::Msg
  {
    public:
      typedef franka_gripper::StopActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef franka_gripper::StopActionResult _action_result_type;
      _action_result_type action_result;
      typedef franka_gripper::StopActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    StopAction():
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

    virtual const char * getType() override { return "franka_gripper/StopAction"; };
    virtual const char * getMD5() override { return "f37964fcdec026d9507d088d32b65b38"; };

  };

}
#endif
