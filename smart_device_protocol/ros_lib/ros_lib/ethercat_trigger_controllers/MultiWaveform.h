#ifndef _ROS_ethercat_trigger_controllers_MultiWaveform_h
#define _ROS_ethercat_trigger_controllers_MultiWaveform_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ethercat_trigger_controllers/MultiWaveformTransition.h"

namespace ethercat_trigger_controllers
{

  class MultiWaveform : public ros::Msg
  {
    public:
      typedef float _period_type;
      _period_type period;
      typedef float _zero_offset_type;
      _zero_offset_type zero_offset;
      uint32_t transitions_length;
      typedef ethercat_trigger_controllers::MultiWaveformTransition _transitions_type;
      _transitions_type st_transitions;
      _transitions_type * transitions;

    MultiWaveform():
      period(0),
      zero_offset(0),
      transitions_length(0), st_transitions(), transitions(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->period);
      offset += serializeAvrFloat64(outbuffer + offset, this->zero_offset);
      *(outbuffer + offset + 0) = (this->transitions_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->transitions_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->transitions_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->transitions_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->transitions_length);
      for( uint32_t i = 0; i < transitions_length; i++){
      offset += this->transitions[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->period));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->zero_offset));
      uint32_t transitions_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      transitions_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      transitions_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      transitions_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->transitions_length);
      if(transitions_lengthT > transitions_length)
        this->transitions = (ethercat_trigger_controllers::MultiWaveformTransition*)realloc(this->transitions, transitions_lengthT * sizeof(ethercat_trigger_controllers::MultiWaveformTransition));
      transitions_length = transitions_lengthT;
      for( uint32_t i = 0; i < transitions_length; i++){
      offset += this->st_transitions.deserialize(inbuffer + offset);
        memcpy( &(this->transitions[i]), &(this->st_transitions), sizeof(ethercat_trigger_controllers::MultiWaveformTransition));
      }
     return offset;
    }

    virtual const char * getType() override { return "ethercat_trigger_controllers/MultiWaveform"; };
    virtual const char * getMD5() override { return "6a8e166563c159e73f391a302e7b37f6"; };

  };

}
#endif
