#ifndef _ROS_franka_gripper_GraspAction_h
#define _ROS_franka_gripper_GraspAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "franka_gripper/GraspActionGoal.h"
#include "franka_gripper/GraspActionResult.h"
#include "franka_gripper/GraspActionFeedback.h"

namespace franka_gripper
{

  class GraspAction : public ros::Msg
  {
    public:
      typedef franka_gripper::GraspActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef franka_gripper::GraspActionResult _action_result_type;
      _action_result_type action_result;
      typedef franka_gripper::GraspActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    GraspAction():
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

    virtual const char * getType() override { return "franka_gripper/GraspAction"; };
    virtual const char * getMD5() override { return "6f89449359949eaf1f0fef57b7e0db2a"; };

  };

}
#endif
