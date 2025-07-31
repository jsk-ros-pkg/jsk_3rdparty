#ifndef _ROS_naoqi_bridge_msgs_SpeechWithFeedbackGoal_h
#define _ROS_naoqi_bridge_msgs_SpeechWithFeedbackGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace naoqi_bridge_msgs
{

  class SpeechWithFeedbackGoal : public ros::Msg
  {
    public:
      typedef const char* _say_type;
      _say_type say;

    SpeechWithFeedbackGoal():
      say("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_say = strlen(this->say);
      varToArr(outbuffer + offset, length_say);
      offset += 4;
      memcpy(outbuffer + offset, this->say, length_say);
      offset += length_say;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_say;
      arrToVar(length_say, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_say; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_say-1]=0;
      this->say = (char *)(inbuffer + offset-1);
      offset += length_say;
     return offset;
    }

    virtual const char * getType() override { return "naoqi_bridge_msgs/SpeechWithFeedbackGoal"; };
    virtual const char * getMD5() override { return "331898fd34308d7c3706d43ca7f6e377"; };

  };

}
#endif
