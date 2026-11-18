#ifndef _ROS_switchbot_ros_SwitchBotCommandAction_h
#define _ROS_switchbot_ros_SwitchBotCommandAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "switchbot_ros/SwitchBotCommandActionGoal.h"
#include "switchbot_ros/SwitchBotCommandActionResult.h"
#include "switchbot_ros/SwitchBotCommandActionFeedback.h"

namespace switchbot_ros
{

  class SwitchBotCommandAction : public ros::Msg
  {
    public:
      typedef switchbot_ros::SwitchBotCommandActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef switchbot_ros::SwitchBotCommandActionResult _action_result_type;
      _action_result_type action_result;
      typedef switchbot_ros::SwitchBotCommandActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    SwitchBotCommandAction():
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

    virtual const char * getType() override { return "switchbot_ros/SwitchBotCommandAction"; };
    virtual const char * getMD5() override { return "bb09a80982618957781a83067b718d43"; };

  };

}
#endif
