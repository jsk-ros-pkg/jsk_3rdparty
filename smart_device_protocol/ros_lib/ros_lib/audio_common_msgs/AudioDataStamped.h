#ifndef _ROS_audio_common_msgs_AudioDataStamped_h
#define _ROS_audio_common_msgs_AudioDataStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "audio_common_msgs/AudioData.h"

namespace audio_common_msgs
{

  class AudioDataStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef audio_common_msgs::AudioData _audio_type;
      _audio_type audio;

    AudioDataStamped():
      header(),
      audio()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->audio.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->audio.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "audio_common_msgs/AudioDataStamped"; };
    virtual const char * getMD5() override { return "3cdd84a06846af0dca4d0434908f9d96"; };

  };

}
#endif
