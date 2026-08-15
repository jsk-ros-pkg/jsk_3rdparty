#ifndef _ROS_naoqi_bridge_msgs_BlinkResult_h
#define _ROS_naoqi_bridge_msgs_BlinkResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace naoqi_bridge_msgs
{

  class BlinkResult : public ros::Msg
  {
    public:
      typedef bool _still_blinking_type;
      _still_blinking_type still_blinking;

    BlinkResult():
      still_blinking(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_still_blinking;
      u_still_blinking.real = this->still_blinking;
      *(outbuffer + offset + 0) = (u_still_blinking.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->still_blinking);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_still_blinking;
      u_still_blinking.base = 0;
      u_still_blinking.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->still_blinking = u_still_blinking.real;
      offset += sizeof(this->still_blinking);
     return offset;
    }

    virtual const char * getType() override { return "naoqi_bridge_msgs/BlinkResult"; };
    virtual const char * getMD5() override { return "53e041b81450f9247036f13b3c0bf822"; };

  };

}
#endif
