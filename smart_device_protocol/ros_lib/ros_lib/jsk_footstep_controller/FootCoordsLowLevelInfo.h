#ifndef _ROS_jsk_footstep_controller_FootCoordsLowLevelInfo_h
#define _ROS_jsk_footstep_controller_FootCoordsLowLevelInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace jsk_footstep_controller
{

  class FootCoordsLowLevelInfo : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _raw_lleg_force_type;
      _raw_lleg_force_type raw_lleg_force;
      typedef float _raw_rleg_force_type;
      _raw_rleg_force_type raw_rleg_force;
      typedef float _filtered_lleg_force_type;
      _filtered_lleg_force_type filtered_lleg_force;
      typedef float _filtered_rleg_force_type;
      _filtered_rleg_force_type filtered_rleg_force;
      typedef float _threshold_type;
      _threshold_type threshold;

    FootCoordsLowLevelInfo():
      header(),
      raw_lleg_force(0),
      raw_rleg_force(0),
      filtered_lleg_force(0),
      filtered_rleg_force(0),
      threshold(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->raw_lleg_force);
      offset += serializeAvrFloat64(outbuffer + offset, this->raw_rleg_force);
      offset += serializeAvrFloat64(outbuffer + offset, this->filtered_lleg_force);
      offset += serializeAvrFloat64(outbuffer + offset, this->filtered_rleg_force);
      offset += serializeAvrFloat64(outbuffer + offset, this->threshold);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->raw_lleg_force));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->raw_rleg_force));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->filtered_lleg_force));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->filtered_rleg_force));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->threshold));
     return offset;
    }

    virtual const char * getType() override { return "jsk_footstep_controller/FootCoordsLowLevelInfo"; };
    virtual const char * getMD5() override { return "f03f7aaafde613e7d2247f1ee6314403"; };

  };

}
#endif
