#ifndef _ROS_jsk_footstep_controller_LookAroundGroundAction_h
#define _ROS_jsk_footstep_controller_LookAroundGroundAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "jsk_footstep_controller/LookAroundGroundActionGoal.h"
#include "jsk_footstep_controller/LookAroundGroundActionResult.h"
#include "jsk_footstep_controller/LookAroundGroundActionFeedback.h"

namespace jsk_footstep_controller
{

  class LookAroundGroundAction : public ros::Msg
  {
    public:
      typedef jsk_footstep_controller::LookAroundGroundActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef jsk_footstep_controller::LookAroundGroundActionResult _action_result_type;
      _action_result_type action_result;
      typedef jsk_footstep_controller::LookAroundGroundActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    LookAroundGroundAction():
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

    virtual const char * getType() override { return "jsk_footstep_controller/LookAroundGroundAction"; };
    virtual const char * getMD5() override { return "d5a016b49f278075666fbc901debbd08"; };

  };

}
#endif
