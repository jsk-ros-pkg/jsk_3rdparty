#ifndef _ROS_tf2_web_republisher_TFSubscriptionAction_h
#define _ROS_tf2_web_republisher_TFSubscriptionAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "tf2_web_republisher/TFSubscriptionActionGoal.h"
#include "tf2_web_republisher/TFSubscriptionActionResult.h"
#include "tf2_web_republisher/TFSubscriptionActionFeedback.h"

namespace tf2_web_republisher
{

  class TFSubscriptionAction : public ros::Msg
  {
    public:
      typedef tf2_web_republisher::TFSubscriptionActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef tf2_web_republisher::TFSubscriptionActionResult _action_result_type;
      _action_result_type action_result;
      typedef tf2_web_republisher::TFSubscriptionActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    TFSubscriptionAction():
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

    virtual const char * getType() override { return "tf2_web_republisher/TFSubscriptionAction"; };
    virtual const char * getMD5() override { return "15787ffd6a2492c0022abe990c898794"; };

  };

}
#endif
