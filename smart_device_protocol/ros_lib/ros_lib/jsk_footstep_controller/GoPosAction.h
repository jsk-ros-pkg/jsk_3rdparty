#ifndef _ROS_jsk_footstep_controller_GoPosAction_h
#define _ROS_jsk_footstep_controller_GoPosAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "jsk_footstep_controller/GoPosActionGoal.h"
#include "jsk_footstep_controller/GoPosActionResult.h"
#include "jsk_footstep_controller/GoPosActionFeedback.h"

namespace jsk_footstep_controller
{

  class GoPosAction : public ros::Msg
  {
    public:
      typedef jsk_footstep_controller::GoPosActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef jsk_footstep_controller::GoPosActionResult _action_result_type;
      _action_result_type action_result;
      typedef jsk_footstep_controller::GoPosActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    GoPosAction():
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

    virtual const char * getType() override { return "jsk_footstep_controller/GoPosAction"; };
    virtual const char * getMD5() override { return "950fe2e36bc3f398cac7bf97f2a7fcb8"; };

  };

}
#endif
