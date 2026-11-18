#ifndef _ROS_SERVICE_AddOffset_h
#define _ROS_SERVICE_AddOffset_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace microstrain_3dmgx2_imu
{

static const char ADDOFFSET[] = "microstrain_3dmgx2_imu/AddOffset";

  class AddOffsetRequest : public ros::Msg
  {
    public:
      typedef float _add_offset_type;
      _add_offset_type add_offset;

    AddOffsetRequest():
      add_offset(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->add_offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->add_offset));
     return offset;
    }

    virtual const char * getType() override { return ADDOFFSET; };
    virtual const char * getMD5() override { return "10fe27c5d4591264b9d05acc7497a18a"; };

  };

  class AddOffsetResponse : public ros::Msg
  {
    public:
      typedef float _total_offset_type;
      _total_offset_type total_offset;

    AddOffsetResponse():
      total_offset(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->total_offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->total_offset));
     return offset;
    }

    virtual const char * getType() override { return ADDOFFSET; };
    virtual const char * getMD5() override { return "5dea42ce4656fada4736ce3508b56aca"; };

  };

  class AddOffset {
    public:
    typedef AddOffsetRequest Request;
    typedef AddOffsetResponse Response;
  };

}
#endif
