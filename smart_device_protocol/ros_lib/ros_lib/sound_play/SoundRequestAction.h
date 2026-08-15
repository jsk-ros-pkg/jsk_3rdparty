#ifndef _ROS_sound_play_SoundRequestAction_h
#define _ROS_sound_play_SoundRequestAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sound_play/SoundRequestActionGoal.h"
#include "sound_play/SoundRequestActionResult.h"
#include "sound_play/SoundRequestActionFeedback.h"

namespace sound_play
{

  class SoundRequestAction : public ros::Msg
  {
    public:
      typedef sound_play::SoundRequestActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef sound_play::SoundRequestActionResult _action_result_type;
      _action_result_type action_result;
      typedef sound_play::SoundRequestActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    SoundRequestAction():
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

    virtual const char * getType() override { return "sound_play/SoundRequestAction"; };
    virtual const char * getMD5() override { return "f990cf5de6a2f8e514e825b2c1f4810b"; };

  };

}
#endif
