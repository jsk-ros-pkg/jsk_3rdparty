#ifndef _ROS_fetch_auto_dock_msgs_UndockGoal_h
#define _ROS_fetch_auto_dock_msgs_UndockGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace fetch_auto_dock_msgs
{

  class UndockGoal : public ros::Msg
  {
    public:
      typedef bool _rotate_in_place_type;
      _rotate_in_place_type rotate_in_place;

    UndockGoal():
      rotate_in_place(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_rotate_in_place;
      u_rotate_in_place.real = this->rotate_in_place;
      *(outbuffer + offset + 0) = (u_rotate_in_place.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->rotate_in_place);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_rotate_in_place;
      u_rotate_in_place.base = 0;
      u_rotate_in_place.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->rotate_in_place = u_rotate_in_place.real;
      offset += sizeof(this->rotate_in_place);
     return offset;
    }

    virtual const char * getType() override { return "fetch_auto_dock_msgs/UndockGoal"; };
    virtual const char * getMD5() override { return "dc4d50a0ddde1312dc506a49b185c018"; };

  };

}
#endif
