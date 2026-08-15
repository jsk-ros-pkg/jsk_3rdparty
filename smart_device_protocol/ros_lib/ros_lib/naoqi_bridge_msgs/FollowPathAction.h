#ifndef _ROS_naoqi_bridge_msgs_FollowPathAction_h
#define _ROS_naoqi_bridge_msgs_FollowPathAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "naoqi_bridge_msgs/FollowPathActionGoal.h"
#include "naoqi_bridge_msgs/FollowPathActionResult.h"
#include "naoqi_bridge_msgs/FollowPathActionFeedback.h"

namespace naoqi_bridge_msgs
{

  class FollowPathAction : public ros::Msg
  {
    public:
      typedef naoqi_bridge_msgs::FollowPathActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef naoqi_bridge_msgs::FollowPathActionResult _action_result_type;
      _action_result_type action_result;
      typedef naoqi_bridge_msgs::FollowPathActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    FollowPathAction():
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

    virtual const char * getType() override { return "naoqi_bridge_msgs/FollowPathAction"; };
    virtual const char * getMD5() override { return "98958d560f45913f6e3143ad99e2fcf0"; };

  };

}
#endif
