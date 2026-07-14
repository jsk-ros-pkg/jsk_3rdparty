#ifndef _ROS_grasping_msgs_GraspPlanningAction_h
#define _ROS_grasping_msgs_GraspPlanningAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "grasping_msgs/GraspPlanningActionGoal.h"
#include "grasping_msgs/GraspPlanningActionResult.h"
#include "grasping_msgs/GraspPlanningActionFeedback.h"

namespace grasping_msgs
{

  class GraspPlanningAction : public ros::Msg
  {
    public:
      typedef grasping_msgs::GraspPlanningActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef grasping_msgs::GraspPlanningActionResult _action_result_type;
      _action_result_type action_result;
      typedef grasping_msgs::GraspPlanningActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    GraspPlanningAction():
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

    virtual const char * getType() override { return "grasping_msgs/GraspPlanningAction"; };
    virtual const char * getMD5() override { return "7133eaf5c7bbf3f1f2b109ce543a58b0"; };

  };

}
#endif
