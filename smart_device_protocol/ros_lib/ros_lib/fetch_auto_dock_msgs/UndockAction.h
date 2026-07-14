#ifndef _ROS_fetch_auto_dock_msgs_UndockAction_h
#define _ROS_fetch_auto_dock_msgs_UndockAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "fetch_auto_dock_msgs/UndockActionGoal.h"
#include "fetch_auto_dock_msgs/UndockActionResult.h"
#include "fetch_auto_dock_msgs/UndockActionFeedback.h"

namespace fetch_auto_dock_msgs
{

  class UndockAction : public ros::Msg
  {
    public:
      typedef fetch_auto_dock_msgs::UndockActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef fetch_auto_dock_msgs::UndockActionResult _action_result_type;
      _action_result_type action_result;
      typedef fetch_auto_dock_msgs::UndockActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    UndockAction():
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

    virtual const char * getType() override { return "fetch_auto_dock_msgs/UndockAction"; };
    virtual const char * getMD5() override { return "67f75452c196985277234c9f084093d5"; };

  };

}
#endif
