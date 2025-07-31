#ifndef _ROS_sound_play_SoundRequestGoal_h
#define _ROS_sound_play_SoundRequestGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sound_play/SoundRequest.h"

namespace sound_play
{

  class SoundRequestGoal : public ros::Msg
  {
    public:
      typedef sound_play::SoundRequest _sound_request_type;
      _sound_request_type sound_request;

    SoundRequestGoal():
      sound_request()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->sound_request.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->sound_request.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "sound_play/SoundRequestGoal"; };
    virtual const char * getMD5() override { return "3bd5e9e7f60b85cc6f1b7662fe6e1903"; };

  };

}
#endif
