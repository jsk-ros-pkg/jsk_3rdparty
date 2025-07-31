#ifndef _ROS_moveit_msgs_MoveGroupSequenceAction_h
#define _ROS_moveit_msgs_MoveGroupSequenceAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "moveit_msgs/MoveGroupSequenceActionGoal.h"
#include "moveit_msgs/MoveGroupSequenceActionResult.h"
#include "moveit_msgs/MoveGroupSequenceActionFeedback.h"

namespace moveit_msgs
{

  class MoveGroupSequenceAction : public ros::Msg
  {
    public:
      typedef moveit_msgs::MoveGroupSequenceActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef moveit_msgs::MoveGroupSequenceActionResult _action_result_type;
      _action_result_type action_result;
      typedef moveit_msgs::MoveGroupSequenceActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    MoveGroupSequenceAction():
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

    virtual const char * getType() override { return "moveit_msgs/MoveGroupSequenceAction"; };
    virtual const char * getMD5() override { return "0d3375f4fb59ed3ba86fa746e5d8e219"; };

  };

}
#endif
