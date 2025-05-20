#ifndef _ROS_pr2_msgs_PeriodicCmd_h
#define _ROS_pr2_msgs_PeriodicCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace pr2_msgs
{

  class PeriodicCmd : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _profile_type;
      _profile_type profile;
      typedef float _period_type;
      _period_type period;
      typedef float _amplitude_type;
      _amplitude_type amplitude;
      typedef float _offset_type;
      _offset_type offset;

    PeriodicCmd():
      header(),
      profile(""),
      period(0),
      amplitude(0),
      offset(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_profile = strlen(this->profile);
      varToArr(outbuffer + offset, length_profile);
      offset += 4;
      memcpy(outbuffer + offset, this->profile, length_profile);
      offset += length_profile;
      offset += serializeAvrFloat64(outbuffer + offset, this->period);
      offset += serializeAvrFloat64(outbuffer + offset, this->amplitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_profile;
      arrToVar(length_profile, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_profile; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_profile-1]=0;
      this->profile = (char *)(inbuffer + offset-1);
      offset += length_profile;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->period));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->amplitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->offset));
     return offset;
    }

    virtual const char * getType() override { return "pr2_msgs/PeriodicCmd"; };
    virtual const char * getMD5() override { return "95ab7e548e3d4274f83393129dd96c2e"; };

  };

}
#endif
