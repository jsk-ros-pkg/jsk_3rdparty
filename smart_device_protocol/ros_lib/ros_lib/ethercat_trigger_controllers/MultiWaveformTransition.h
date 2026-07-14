#ifndef _ROS_ethercat_trigger_controllers_MultiWaveformTransition_h
#define _ROS_ethercat_trigger_controllers_MultiWaveformTransition_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ethercat_trigger_controllers
{

  class MultiWaveformTransition : public ros::Msg
  {
    public:
      typedef float _time_type;
      _time_type time;
      typedef uint32_t _value_type;
      _value_type value;
      typedef const char* _topic_type;
      _topic_type topic;

    MultiWaveformTransition():
      time(0),
      value(0),
      topic("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->time);
      *(outbuffer + offset + 0) = (this->value >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->value >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->value >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->value >> (8 * 3)) & 0xFF;
      offset += sizeof(this->value);
      uint32_t length_topic = strlen(this->topic);
      varToArr(outbuffer + offset, length_topic);
      offset += 4;
      memcpy(outbuffer + offset, this->topic, length_topic);
      offset += length_topic;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->time));
      this->value =  ((uint32_t) (*(inbuffer + offset)));
      this->value |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->value |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->value |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->value);
      uint32_t length_topic;
      arrToVar(length_topic, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_topic; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_topic-1]=0;
      this->topic = (char *)(inbuffer + offset-1);
      offset += length_topic;
     return offset;
    }

    virtual const char * getType() override { return "ethercat_trigger_controllers/MultiWaveformTransition"; };
    virtual const char * getMD5() override { return "bdd47e5d1c3d77473af2df9833a0608a"; };

  };

}
#endif
