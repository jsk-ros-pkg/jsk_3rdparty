#ifndef _ROS_fetch_driver_msgs_DisableChargingAction_h
#define _ROS_fetch_driver_msgs_DisableChargingAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "fetch_driver_msgs/DisableChargingActionGoal.h"
#include "fetch_driver_msgs/DisableChargingActionResult.h"
#include "fetch_driver_msgs/DisableChargingActionFeedback.h"

namespace fetch_driver_msgs
{

  class DisableChargingAction : public ros::Msg
  {
    public:
      typedef fetch_driver_msgs::DisableChargingActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef fetch_driver_msgs::DisableChargingActionResult _action_result_type;
      _action_result_type action_result;
      typedef fetch_driver_msgs::DisableChargingActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    DisableChargingAction():
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

    virtual const char * getType() override { return "fetch_driver_msgs/DisableChargingAction"; };
    virtual const char * getMD5() override { return "779104bea58517403422a5cae55a1bb1"; };

  };

}
#endif
