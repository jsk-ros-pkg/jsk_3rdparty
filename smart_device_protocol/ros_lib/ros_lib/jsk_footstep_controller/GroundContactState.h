#ifndef _ROS_jsk_footstep_controller_GroundContactState_h
#define _ROS_jsk_footstep_controller_GroundContactState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace jsk_footstep_controller
{

  class GroundContactState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _contact_state_type;
      _contact_state_type contact_state;
      typedef float _error_pitch_angle_type;
      _error_pitch_angle_type error_pitch_angle;
      typedef float _error_roll_angle_type;
      _error_roll_angle_type error_roll_angle;
      typedef float _error_yaw_angle_type;
      _error_yaw_angle_type error_yaw_angle;
      typedef float _error_z_type;
      _error_z_type error_z;
      enum { CONTACT_BOTH_GROUND = 1 };
      enum { CONTACT_AIR = 2 };
      enum { CONTACT_LLEG_GROUND = 3 };
      enum { CONTACT_RLEG_GROUND = 4 };
      enum { CONTACT_UNSTABLE = 5 };

    GroundContactState():
      header(),
      contact_state(0),
      error_pitch_angle(0),
      error_roll_angle(0),
      error_yaw_angle(0),
      error_z(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->contact_state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->contact_state);
      offset += serializeAvrFloat64(outbuffer + offset, this->error_pitch_angle);
      offset += serializeAvrFloat64(outbuffer + offset, this->error_roll_angle);
      offset += serializeAvrFloat64(outbuffer + offset, this->error_yaw_angle);
      offset += serializeAvrFloat64(outbuffer + offset, this->error_z);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->contact_state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->contact_state);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->error_pitch_angle));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->error_roll_angle));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->error_yaw_angle));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->error_z));
     return offset;
    }

    virtual const char * getType() override { return "jsk_footstep_controller/GroundContactState"; };
    virtual const char * getMD5() override { return "da0f3906e0a6eafe324ba582440493ea"; };

  };

}
#endif
