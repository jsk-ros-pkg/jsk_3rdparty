#ifndef _ROS_jsk_footstep_controller_GoPosFeedback_h
#define _ROS_jsk_footstep_controller_GoPosFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace jsk_footstep_controller
{

  class GoPosFeedback : public ros::Msg
  {
    public:
      typedef uint8_t _status_type;
      _status_type status;
      typedef int32_t _remaining_steps_type;
      _remaining_steps_type remaining_steps;
      enum { PRE_PLANNING = 0 };
      enum { PLANNING = 1 };
      enum { WALKING = 2 };
      enum { WAITING = 3 };
      enum { FINISH = 4 };

    GoPosFeedback():
      status(0),
      remaining_steps(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      union {
        int32_t real;
        uint32_t base;
      } u_remaining_steps;
      u_remaining_steps.real = this->remaining_steps;
      *(outbuffer + offset + 0) = (u_remaining_steps.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_remaining_steps.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_remaining_steps.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_remaining_steps.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->remaining_steps);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->status);
      union {
        int32_t real;
        uint32_t base;
      } u_remaining_steps;
      u_remaining_steps.base = 0;
      u_remaining_steps.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_remaining_steps.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_remaining_steps.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_remaining_steps.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->remaining_steps = u_remaining_steps.real;
      offset += sizeof(this->remaining_steps);
     return offset;
    }

    virtual const char * getType() override { return "jsk_footstep_controller/GoPosFeedback"; };
    virtual const char * getMD5() override { return "4b0fee2db6d7d95642702999456a0721"; };

  };

}
#endif
