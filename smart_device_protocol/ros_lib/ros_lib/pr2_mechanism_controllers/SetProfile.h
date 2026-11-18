#ifndef _ROS_SERVICE_SetProfile_h
#define _ROS_SERVICE_SetProfile_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr2_mechanism_controllers
{

static const char SETPROFILE[] = "pr2_mechanism_controllers/SetProfile";

  class SetProfileRequest : public ros::Msg
  {
    public:
      typedef float _UpperTurnaround_type;
      _UpperTurnaround_type UpperTurnaround;
      typedef float _LowerTurnaround_type;
      _LowerTurnaround_type LowerTurnaround;
      typedef float _upperDecelBuffer_type;
      _upperDecelBuffer_type upperDecelBuffer;
      typedef float _lowerDecelBuffer_type;
      _lowerDecelBuffer_type lowerDecelBuffer;
      typedef float _profile_type;
      _profile_type profile;
      typedef float _period_type;
      _period_type period;
      typedef float _amplitude_type;
      _amplitude_type amplitude;
      typedef float _offset_type;
      _offset_type offset;

    SetProfileRequest():
      UpperTurnaround(0),
      LowerTurnaround(0),
      upperDecelBuffer(0),
      lowerDecelBuffer(0),
      profile(0),
      period(0),
      amplitude(0),
      offset(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->UpperTurnaround);
      offset += serializeAvrFloat64(outbuffer + offset, this->LowerTurnaround);
      offset += serializeAvrFloat64(outbuffer + offset, this->upperDecelBuffer);
      offset += serializeAvrFloat64(outbuffer + offset, this->lowerDecelBuffer);
      offset += serializeAvrFloat64(outbuffer + offset, this->profile);
      offset += serializeAvrFloat64(outbuffer + offset, this->period);
      offset += serializeAvrFloat64(outbuffer + offset, this->amplitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->UpperTurnaround));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->LowerTurnaround));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->upperDecelBuffer));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->lowerDecelBuffer));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->profile));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->period));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->amplitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->offset));
     return offset;
    }

    virtual const char * getType() override { return SETPROFILE; };
    virtual const char * getMD5() override { return "309001fc196b0094f23b1ae2bd672fb2"; };

  };

  class SetProfileResponse : public ros::Msg
  {
    public:
      typedef float _time_type;
      _time_type time;

    SetProfileResponse():
      time(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->time);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->time));
     return offset;
    }

    virtual const char * getType() override { return SETPROFILE; };
    virtual const char * getMD5() override { return "be5310e7aa4c90cdee120add91648cee"; };

  };

  class SetProfile {
    public:
    typedef SetProfileRequest Request;
    typedef SetProfileResponse Response;
  };

}
#endif
