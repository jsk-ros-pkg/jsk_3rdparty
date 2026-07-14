#ifndef _ROS_pr2_controllers_msgs_Pr2GripperCommandAction_h
#define _ROS_pr2_controllers_msgs_Pr2GripperCommandAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr2_controllers_msgs/Pr2GripperCommandActionGoal.h"
#include "pr2_controllers_msgs/Pr2GripperCommandActionResult.h"
#include "pr2_controllers_msgs/Pr2GripperCommandActionFeedback.h"

namespace pr2_controllers_msgs
{

  class Pr2GripperCommandAction : public ros::Msg
  {
    public:
      typedef pr2_controllers_msgs::Pr2GripperCommandActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef pr2_controllers_msgs::Pr2GripperCommandActionResult _action_result_type;
      _action_result_type action_result;
      typedef pr2_controllers_msgs::Pr2GripperCommandActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    Pr2GripperCommandAction():
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

    virtual const char * getType() override { return "pr2_controllers_msgs/Pr2GripperCommandAction"; };
    virtual const char * getMD5() override { return "950b2a6ebe831f5d4f4ceaba3d8be01e"; };

  };

}
#endif
