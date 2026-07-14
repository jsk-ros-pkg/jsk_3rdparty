#ifndef _ROS_franka_gripper_MoveAction_h
#define _ROS_franka_gripper_MoveAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "franka_gripper/MoveActionGoal.h"
#include "franka_gripper/MoveActionResult.h"
#include "franka_gripper/MoveActionFeedback.h"

namespace franka_gripper
{

  class MoveAction : public ros::Msg
  {
    public:
      typedef franka_gripper::MoveActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef franka_gripper::MoveActionResult _action_result_type;
      _action_result_type action_result;
      typedef franka_gripper::MoveActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    MoveAction():
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

    virtual const char * getType() override { return "franka_gripper/MoveAction"; };
    virtual const char * getMD5() override { return "85e8e11f508e446b706022926779afee"; };

  };

}
#endif
