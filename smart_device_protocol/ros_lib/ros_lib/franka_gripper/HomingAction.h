#ifndef _ROS_franka_gripper_HomingAction_h
#define _ROS_franka_gripper_HomingAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "franka_gripper/HomingActionGoal.h"
#include "franka_gripper/HomingActionResult.h"
#include "franka_gripper/HomingActionFeedback.h"

namespace franka_gripper
{

  class HomingAction : public ros::Msg
  {
    public:
      typedef franka_gripper::HomingActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef franka_gripper::HomingActionResult _action_result_type;
      _action_result_type action_result;
      typedef franka_gripper::HomingActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    HomingAction():
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

    virtual const char * getType() override { return "franka_gripper/HomingAction"; };
    virtual const char * getMD5() override { return "f37964fcdec026d9507d088d32b65b38"; };

  };

}
#endif
