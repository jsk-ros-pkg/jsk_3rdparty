#ifndef _ROS_moveit_msgs_PlaceAction_h
#define _ROS_moveit_msgs_PlaceAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "moveit_msgs/PlaceActionGoal.h"
#include "moveit_msgs/PlaceActionResult.h"
#include "moveit_msgs/PlaceActionFeedback.h"

namespace moveit_msgs
{

  class PlaceAction : public ros::Msg
  {
    public:
      typedef moveit_msgs::PlaceActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef moveit_msgs::PlaceActionResult _action_result_type;
      _action_result_type action_result;
      typedef moveit_msgs::PlaceActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    PlaceAction():
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

    virtual const char * getType() override { return "moveit_msgs/PlaceAction"; };
    virtual const char * getMD5() override { return "a251d4a0c56193b6e1b76a1ca20de499"; };

  };

}
#endif
