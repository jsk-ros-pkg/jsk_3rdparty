#ifndef _ROS_roseus_smach_Multiply10Action_h
#define _ROS_roseus_smach_Multiply10Action_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "roseus_smach/Multiply10ActionGoal.h"
#include "roseus_smach/Multiply10ActionResult.h"
#include "roseus_smach/Multiply10ActionFeedback.h"

namespace roseus_smach
{

  class Multiply10Action : public ros::Msg
  {
    public:
      typedef roseus_smach::Multiply10ActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef roseus_smach::Multiply10ActionResult _action_result_type;
      _action_result_type action_result;
      typedef roseus_smach::Multiply10ActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    Multiply10Action():
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

    virtual const char * getType() override { return "roseus_smach/Multiply10Action"; };
    virtual const char * getMD5() override { return "649e5cd3d0369bfd72c1f24aed2b51ea"; };

  };

}
#endif
